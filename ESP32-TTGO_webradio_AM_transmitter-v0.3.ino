/*========================================================================
test for webradio receiver AM broadcast transmitter

-> ESP32 TTGO t-display ST7789V SPI bus TFT
   default code https://github.com/Xinyuan-LilyGO/TTGO-T-Display/blob/master/TFT_eSPI/examples/FactoryTest/FactoryTest.ino

 IDE version 1.8.16 with 
 Expressif Sytems (ESP32) version 2.0.3  
 => Audio.h v2.0.6

//------------------------------------------------------------------------
//  Board            : "ESP32 Dev Module"
//------------------------------------------------------------------------
//  Upload Speed     : "921600"
//  CPU Frequency    : "240MHz (WiFi/BT)"
//  Flash Mode       : "QIO"
//  Partition Scheme : "Huge APP (3MB APP no OTA/1MB SPIFFS)"
//  PSRAM            : "Disabled"

  ESP32-TTGO_T_Display pinup
-----------------------------

GPIO14  => ADC_EN      // ADC_EN is the ADC detection enable port
GPIO34  => ADC_PIN     // specific for ESP32-TTGO

GPIO25  => DAC1 output webradio output
GPIO26  => DAC2 output

GPIO33  => analog input
GPIO32  => RF output

GPIO35  => push-button station +
GPIO0   => push-button station -

//========================================================================

_________________________________________________________________
|                                                               |
|       author : Philippe de Craene <dcphilippe@yahoo.fr>       |
|                                                               |
_________________________________________________________________

version history
---------------

v0    24 oct 25   - first test webradio to AM transmitter
v0.2  26 oct 25   - add mDNS and battery voltage measure
v0.3  26 oct 25   - give the choice of the rf frequency

*/

// parameters
//-----------------
const String VERSION = "v0.3";    // code version
int rfFreq = 240;                 // carrier frequency in kHz
const char* dnshost = "tsf";      // DNS name

byte volume = 21;                 // initial audio volume
int radioPlayingDelay = 120;      // radio autostop in minutes
int maxTitleLenght = 40;          // max lenght of radio title

// the RF modulation
//------------------
#define rfIn       33     // ADC1_CH4 - no ADC2 with wifi
#define rfOut      32
#define PWM_channel 0     // pwm channel - choise = [0 - 15]
#define PWM_Res     8     // [0 - 255] 8 bits resolution - choise = [1 - 16] 
/* resolution depends of the PWM frequency:
 *  500kHz - 1 MHz => 6 bits
 *  250kHz - 500kHz => 7 bits
 *  125kHz - 250kHz => 8 bits
 *   60kHz - 125kHz => 9 bits
 *   above 60kHz    => 10 bits
 */
hw_timer_t *rfTimer = NULL;  // pointer variable named rfTimer of the type hw_timer_t

// the push-buttons
//------------------
#define pb1      35
#define pb2       0

// Supply voltage monitor
//-----------------------
#define ADC_EN   14       // ADC_EN is the ADC detection enable port
#define ADC_PIN  34       // specific for ESP32-TTGO
float vBatValue = 0;
volatile int vBatBits = 0;
volatile bool vBatFlag = false;

// wifi & web server
//------------------
#include <WiFi.h>
#include <AsyncTCP.h>             // required for ESPAsyncWebServer.h
#include <ESPAsyncWebServer.h>    // https://github.com/me-no-dev/ESPAsyncWebServer
#include <ESPAsyncWiFiManager.h>  // https://github.com/alanswx/ESPAsyncWiFiManager
// web server                     // exemple: https://electropeak.com/learn/create-a-web-server-w-esp32/
AsyncWebServer server(80);        // set web server on port #80
DNSServer dns;
#include <ESPmDNS.h>              // https://techtutorialsx.com/2022/01/24/esp32-webserial-and-mdns/

// display & graphics
//-------------------
//#########################################################################
//###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
//#########################################################################
// !!! SETUP FILE NAMED: User_Setup_Select.h TO REFLECT THE TTGO DISPLAY !!!
// TFT Pins has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
// #define TFT_MOSI        19
// #define TFT_SCLK        18
// #define TFT_CS           5
// #define TFT_DC          16
// #define TFT_RST         23
// #define TFT_BL           4     // Display backlight control pin
#include "TFT_eSPI.h"             // https://github.com/Bodmer/TFT_eSPI
TFT_eSPI tft = TFT_eSPI();        // Use hardware SPI

#define backgroundColor  TFT_BLACK
#define textColor        TFT_GOLD // TFT_WHITE
byte screenRotation = 1;          // 1 or 3

// Audio
//-------
#include "Audio.h"          // https://github.com/schreibfaul1/ESP32-audioI2S
Audio audio(true, I2S_DAC_CHANNEL_RIGHT_EN);  // only DAC1 on GPIO25

// to keep data
//-------------
#include "SPIFFS.h"       // keeps stations.txt file

// EEPROM
#include <Preferences.h>  // https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
Preferences pref;

// radio station list
//-------------------
String stations[] ={
    "live-reflector.ice.infomaniak.ch/rfiafrique-64.mp3",
    "live02.rfi.fr/rfiafrique-64.mp3",
    "icecast.radiofrance.fr/franceculture-lofi.mp3",
    "icecast.radiofrance.fr/franceinter-lofi.mp3",
    "icecast.radiofrance.fr/franceinfo-lofi.mp3",
    "icecast.radiofrance.fr/francemusique-lofi.mp3",
    "stream.srg-ssr.ch/m/rsc_de/mp3_128",
    "ais-sa2.cdnstream1.com/2208_128.mp3",
    "radioseribatu.out.airtime.pro:8000/radioseribatu_a",
    "centova11.instainternet.com:8050/stream",
    " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ",
    " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ",
    " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ",
    " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ",
    " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "
 };
byte radioList = 10;         // total number of stations
int radioNum = 0;            // current radio station index number
String radioName = "", memoRadioName = "";
String radioTitle = "", memoRadioTitle = "";

// other global variables
//-----------------------
String ipAddress = "http://";
String dnsName = "http://";
bool change = true;
int radioCounter = radioPlayingDelay;

// Define the html pages
//----------------------
// example: https://esp32io.com/tutorials/esp32-web-server-multiple-pages

const char view_html[] PROGMEM = R"=====(
    <!DOCTYPE html><html lang=fr-FR><head><title>Emetteur Radio chez Fifi</title>
    <head><meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">

    <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
    .button { background-color: #8A0808; border: none; color: white; padding: 16px 40px;
    text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
    .button2 {background-color: #32CD99;} .button3 {background-color: #08298A;}
    </style></head>

    <BODY><h1>Emetteur Radio chez Fifi</h1>
    %RADIOPLACEHOLDER%
    </body></html>
)=====";

const char confirmdelete_html[] PROGMEM = R"=====(
    <!DOCTYPE html><html lang=fr-FR><head><title>HEmetteur Radio chez Fifi</title>
    <head><meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">

    <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
    .button { background-color: #8A0808; border: none; color: white; padding: 16px 40px;
    text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
    .button2 {background-color: #32CD99;} .button3 {background-color: #08298A;}
    </style></head>

    <BODY><h1>Emetteur Radio chez Fifi</h1>
    %CONFIRMDELETEPLACEHOLDER%
    </body></html>
)=====";

//
// SETUP
//_____________________________________________________________________________________________

void setup() {

// Assign loop2() to core 0
//-------------------------
  xTaskCreatePinnedToCore(
    loop2,     // Function to implement the task
    "loop2",   // Name of the task
    50000,     // Stack size in bytes
    NULL,      // Task input parameter
    5,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );

// start the console
//------------------
   Serial.begin(115200);           // only here to economize memory
   while(!Serial);                 // Wait for user to open terminal
   Serial.println(F("Starting..."));

// start SPIFFS
//-------------
// info: https://www.programmingelectronics.com/spiffs-esp32/
  #ifdef INIT_SPIFFS
   Serial.println(F("Formating SPIFFS then copy SDcard files to it"));
   if(SPIFFS.format()) Serial.println(F("SPIFFS erased successfully."));
  #endif
  SPIFFS.begin(true);

// get the radio stations list
  ListDir(SPIFFS, "/", 0);
  File f = SPIFFS.open("/stations.txt");   // ouverture du fichier en lecture
  if( !f ) {
    Serial.println(F("No file /station.txt, default radio list is used"));
    UpdateRadioList();
  }
  else if( f.size() > 0 ) {
    Serial.println(F("web radio station list:"));
    radioList = 0;
    char c;
    stations[0] = "";
    for( unsigned int i=0; i< f.size(); i++ ) {
      c = f.read();
      if( c != '\r' ) {
        if( c != '\n' ) stations[radioList] += c;
      }
      else {
        Serial.printf("n°%d = ", radioList +1); Serial.println(stations[radioList]);
        radioList++;
        stations[radioList] = "";
      }
    }    // end of for
    f.close();
  }      // end of else f.size() > 0
  else {
    Serial.println(F("File /station.TXT is empty, default radio list is used"));
    f.close();
    UpdateRadioList();
  }
  Serial.printf("total of web radio stations = %d\n", radioList);

// get parameters
//---------------
  pref.begin("parameters", false);
  radioNum = pref.getUChar("radio", 0);
  rfFreq = pref.getInt("freq", rfFreq);
  volume = pref.getUChar("vol", volume);
  screenRotation = pref.getUChar("sr", screenRotation);
  
// start the display
//------------------
  tft.init();
  tft.fillScreen(backgroundColor);
/*
  Normally strings are printed relative to the top left corner but this can be
  changed with the setTextDatum() function. The library has #defines for:

  TL_DATUM = 0 = Top left
  TC_DATUM = 1 = Top centre
  TR_DATUM = 2 = Top right
  ML_DATUM = 3 = Middle left
  MC_DATUM = 4 = Middle centre
  MR_DATUM = 5 = Middle right
  BL_DATUM = 6 = Bottom left
  BC_DATUM = 7 = Bottom centre
  BR_DATUM = 8 = Bottom right

  L_BASELINE =  9 = Left character baseline (Line the 'A' character would sit on)
  C_BASELINE = 10 = Centre character baseline
  R_BASELINE = 11 = Right character baseline
*/
  tft.setRotation(screenRotation);
  tft.setTextSize(2);
  tft.setTextColor(textColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Pour se connecter",  tft.width()/2, tft.height()/2 -40);
  tft.drawString("au Wifi:",           tft.width()/2, tft.height()/2 -20);
  tft.drawString("rechercher WifiAP",  tft.width()/2, tft.height()/2 );
  tft.drawString("puis aller sur :",   tft.width()/2, tft.height()/2 +20);
  tft.drawString("http://192.168.4.1", tft.width()/2, tft.height()/2 +40);

// WiFi Connect
//-------------
  AsyncWiFiManager wifiManager(&server,&dns);
  //wifiManager.resetSettings();  // to reset saved settings
  //wifiManager.setTimeout(240);  // sets timeout until configuration portal gets turned off
  if(!wifiManager.autoConnect("WifiAP")) {
    Serial.println(F("failed to connect and hit timeout: restart device"));
    delay(3000);
    ESP.restart();      // reset and try again, or maybe put it to deep sleep
    delay(5000);
  }

// Connect to Wi-Fi network with SSID and password
  WiFi.setSleep(false);                 // to test as sometime the radio stops
  tft.fillScreen(backgroundColor);
  tft.drawString("Emeteur Radio GO", tft.width()/2, tft.height()/2 -50);
  tft.drawString(VERSION,            tft.width()/2, tft.height()/2 -30);
  tft.drawString("adresse Wifi:",    tft.width()/2, tft.height()/2 );
  ipAddress += WiFi.localIP().toString();
  tft.drawString(ipAddress,          tft.width()/2, tft.height()/2 +20);

// initialize mDNS
  if(!MDNS.begin(dnshost)) Serial.println(F("Error setting up MDNS responder!"));
  else {
    dnsName += String(dnshost) + ".local";
    tft.drawString(dnsName,          tft.width()/2, tft.height()/2 +40);
  }

// prepare the webserver requests
//-------------------------------
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", view_html, processor);
  });
  server.on("/stationsuiv", HTTP_GET, [](AsyncWebServerRequest *request) {
    ChangeRadio(HIGH);
    request->redirect("/");
  });
  server.on("/stationprec", HTTP_GET, [](AsyncWebServerRequest *request) {
    ChangeRadio(LOW);
    request->redirect("/");
  });
  server.on("/confirmdelete", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", confirmdelete_html, processor);
  });
  server.on("/deleteradio", HTTP_GET, [](AsyncWebServerRequest *request) {
    if( radioList > 1 ) {
      radioList--;
      for(byte i= radioNum; i< radioList; i++ ) stations[i] = stations[i+1];
      radioNum--;
      UpdateRadioList();
    }     
    request->redirect("/");
  });
  server.on("/addradio", HTTP_GET, [](AsyncWebServerRequest *request) {
    if( request->hasParam("r")) {
      String newRadio = request->getParam("r")->value();
      if( newRadio.length() > 5 ) {
        radioNum = radioList;
        stations[radioNum] = newRadio;
        Serial.printf("New web radiostation %d: ", radioNum+1); Serial.println(newRadio);
        radioList++;
        UpdateRadioList();
        ListenRadio();
      }
    }
    request->redirect("/");
  });
  server.on("/freqplus", HTTP_GET, [](AsyncWebServerRequest *request) {
    rfFreq += 10;
    if( rfFreq > 300 ) rfFreq = 300;
    pref.putInt("freq", rfFreq);
    RfTransmitter();
    change = true;
    request->redirect("/");
  });
  server.on("/freqmoins", HTTP_GET, [](AsyncWebServerRequest *request) {
    rfFreq -= 10;
    if( rfFreq < 150 ) rfFreq = 150;
    pref.putInt("freq", rfFreq);
    RfTransmitter();
    change = true;
    request->redirect("/");
  });
  server.on("/modulplus", HTTP_GET, [](AsyncWebServerRequest *request) {
    if( ++volume > 21 ) volume = 21;
    pref.putUChar("vol", volume);
    request->redirect("/");
  });
  server.on("/modulmoins", HTTP_GET, [](AsyncWebServerRequest *request) {
    if( --volume < 12 ) volume = 12;
    pref.putUChar("vol", volume);
    request->redirect("/");
  });
  server.on("/rotation", HTTP_GET, [](AsyncWebServerRequest *request) {
    if( screenRotation == 3 ) screenRotation = 1;
    else screenRotation = 3;
    change = true;
    pref.putUChar("sr", screenRotation);
    request->redirect("/");
  });
  server.begin();

// prepare to get battery voltage
//-------------------------------
/* ADC_EN is the ADC detection enable port
  If the USB port is used for power supply, it is turned on by default.
  If it is powered by battery, it needs to be set to high level */
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);

// lower the analogRead definition
  analogReadResolution(9);  // 0-511 instead of 0-4095

// set RF transmitter
  //pinMode(rfOut, OUTPUT_OPEN_DRAIN);
  RfTransmitter();
  ledcWrite(PWM_channel, 128);               // duty cycle of 50%

// timer interrupt (see Arduino example)
  rfTimer = timerBegin(0, 80, true);    // 1st timer of 4 & set 80 divider for prescaler
  timerAttachInterrupt(rfTimer, &onTimer, true);  //  Attach onTimer function to rfTimer.
  // Set alarm to call onTimer function(value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(rfTimer, 150, true);  // 142us gives maxi 7000Hz 
  timerAlarmEnable(rfTimer);

// then to end setup
//------------------
  pinMode(pb1, INPUT_PULLUP);
  pinMode(pb2, INPUT_PULLUP);

  delay(2000);
  tft.fillScreen(backgroundColor);        // BG Color
  change = true;
}      // end of setup

// LOOP
//_____________________________________________________________________________________________

void loop() {

// the following is performed every seconds
//-----------------------------------------
  static unsigned int tempo = 0;
  static unsigned int counter = 0;
  static byte tempoCounter = 0;
  if( millis() < tempo + 500 ) return;
  tempo = millis();

// get battery voltage
  if(counter == 0) vBatFlag = true;
  else if(counter ==1) {
    vBatValue = ((float)vBatBits / 512.0) * 2.0 * 3.3 * 1.1; //(vref / 1000.0);
    DrawDisplay();
  }
  if( ++counter > 180) counter = 0;    // reset every 3 minutes

// check station change
  if(!digitalRead(pb1))      ChangeRadio(LOW);
  else if(!digitalRead(pb2)) ChangeRadio(HIGH);

// draw radio name on tft
  if(change ||(radioName != memoRadioName)||(radioTitle != memoRadioTitle)) {
    memoRadioName = radioName;
    memoRadioTitle = radioTitle;
    if( radioName == "" ) radioName = stations[radioNum];
    int startPos = radioName.lastIndexOf("/");
    int endPos   = radioName.lastIndexOf(".");
    if(startPos > 0) radioName = radioName.substring(startPos +1);
    if(endPos > 0)   radioName = radioName.substring(0, endPos);

    int total = radioTitle.length();
    if(total > maxTitleLenght) {
      String concacRadioTitle = "";
      byte i = 0;
      char c1 = radioTitle.charAt(i);
      if(c1 == ' ') i++;
      while(i < total) {
        char c1 = radioTitle.charAt(i);
        char c2 = radioTitle.charAt(i+1);
        if((c1 == ' ')&&(c2 == ' ')) i++;
        else concacRadioTitle += String(c1);
        i++;
      }
      radioTitle = concacRadioTitle.substring(0, maxTitleLenght);
      endPos = radioTitle.lastIndexOf(" ");
      if(endPos > 0) radioTitle = radioTitle.substring(0, endPos);      
    }
    DrawDisplay();
  }

//  radio autostop
  if( ++tempoCounter > (radioPlayingDelay *60)) {
    tft.fillScreen(backgroundColor);
    tft.drawString("Allez, dodo", tft.width()/2, tft.height()/2);
    delay(2000);
    esp_deep_sleep_start();
  }
}      // end of Loop

//
// LOOP2 the loop2 function also runs forver but as a parallel task
//_____________________________________________________________________________________________

void loop2(void* pvParameters) {

  delay(2000);
  audio.forceMono(true);
  audio.setVolume(volume);          // Set Vol 0...21 or VolumeSteps

// here start the Core 0 loop
//---------------------------
  while(true) {
    audio.loop();
    vTaskDelay(1);      // delay(1);

    if(change) {
      change = false;
      ListenRadio();
      memoRadioName = radioName;
      radioName = "";
      radioTitle = "";
    }
  }    // end of while 
}      // end of loop2

//============================================================================================
// list of functions
//============================================================================================
//
// timer interrupt
//____________________________________________________________________________________________

void ARDUINO_ISR_ATTR onTimer() {
  int duty = analogRead(rfIn) >>1 ;  // /2 = mid duty cycle from 0-511 to 0-255
  ledcWrite(PWM_channel, duty);
  if(vBatFlag) {
    vBatBits = analogRead(ADC_PIN);  // must be done during interruption otherwise abort and reboot
    vBatFlag = false;
  }
}

//
// rfTransmitter : set Carrier (PWM aka LEDC)
//____________________________________________________________________________________________

void RfTransmitter() {
  ledcSetup(PWM_channel, rfFreq *1000, PWM_Res);  // set frequency & resolution
  ledcAttachPin(rfOut, PWM_channel);         // ASSIGN CHANNEL 0 TO GPIO-PIN
}

//
// DrawDisplay() = function to draw the radiostation name
//____________________________________________________________________________________________

void DrawDisplay() {
  tft.setRotation(screenRotation);
  tft.fillScreen(backgroundColor);
  tft.setTextDatum(MC_DATUM);

  tft.setTextSize(2);
  tft.drawString("eRadio en diffusion", tft.width()/2, tft.height()/2 -50);
  String legende = "sur " + String(rfFreq) + "kHz:";
  tft.drawString(legende,   tft.width()/2, tft.height()/2 -30);
  tft.drawString(radioName, tft.width()/2, tft.height()/2 );
  
  tft.setTextSize(1);
  tft.drawString(radioTitle, tft.width()/2, tft.height()/2 +20 );
  legende = "Param:" + dnsName + "     bat:" + String(vBatValue) + "v";
  tft.drawString(legende,   tft.width()/2, tft.height()/2 +50);
}

// ListenRadio() functions to listen the web radio
//____________________________________________________________________________________________

void ListenRadio() {
  //Serial.print(F("ListenRadio() running in core "));  Serial.println(xPortGetCoreID());
  Serial.print(F("web radio station:")); Serial.println(stations[radioNum]);
  audio.connecttohost(stations[radioNum].c_str());
}

void audio_showstation(const char *info){
  Serial.printf("Station: %s\n", info);
  radioName = info;
}

void audio_showstreamtitle(const char *info) {
  Serial.print(F("showstreamtitle=")); Serial.println(info);
  radioTitle = info;
}

void printTitle(const char* info) {
  Serial.print(F("print_title=    ")); Serial.println(info);
}

void printInfo(const char* info) {
   Serial.print(F("print_info=    ")); Serial.println(info);
}

void audio_info(const char *info) {
  Serial.print(F("audio_info=     ")); Serial.println(info);
}

void audio_id3data(const char *info) {
  Serial.print(F("audio_id3data=  ")); Serial.println(info);
}

void audio_bitrate(const char *info) {
    Serial.print(F("bitrate=      ")); Serial.println(info);
}

void audio_commercial(const char *info) {  //duration in sec
    Serial.print(F("commercial=   ")); Serial.println(info);
}

void audio_icyurl(const char *info) {  //homepage
    Serial.print(F("audio_icyurl= ")); Serial.println(info);
}

void audio_lasthost(const char *info) {  //stream URL played
    Serial.print(F("lasthost=     ")); Serial.println(info);
}

// ChangeRadio() change radiostation to listen
//____________________________________________________________________________________________

void ChangeRadio(bool s) {
  if(s) { if(++radioNum >= radioList) radioNum = 0; }
  else  { if(--radioNum < 0) radioNum = radioList -1; }
  pref.putUChar("radio", radioNum);
  change = true;
}

// UpdateRadioList() store radio list to SPIFFS
//____________________________________________________________________________________________

void UpdateRadioList() {
  File f = SPIFFS.open("/stations.txt", FILE_WRITE);
  if(!f) Serial.println(F("Cannot open SPIFFS"));
  else {
    Serial.printf("list of the %d web radio:\n", radioList);
    for( byte i=0; i< radioList; i++ ) {
      Serial.printf("%d = ",i+1); Serial.println(stations[i]);
      f.println(stations[i]);
    }
  }
  f.close();
}

//
// sd card functions
//____________________________________________________________________________________________

void ListDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root) {
    Serial.println(F("Error to open directory"));
    return;
  } 
  if(!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  File file = root.openNextFile();
  while(file) {
    if(file.isDirectory()) {
      Serial.print(F("\tDIR : "));
      Serial.println(file.name());
      if(levels >0) ListDir(fs, file.name(), levels -1);
    }
    else {
      Serial.print(F("\tFILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}      // end of ListDir()

void CopyFile(fs::FS &fs1, const char * path1, fs::FS &fs2, const char * path2) {
  Serial.printf("copying: %s/%s to %s/%s\n", fs1, path1, fs2, path2);
  File file1 = fs1.open(path1, FILE_READ);
  if( fs2.open(path2, FILE_READ)) Serial.printf("%s yet exists, delete it first\n", path2);
  File file2 = fs2.open(path2, FILE_WRITE);

  size_t n;  
  uint8_t buf[64];
  while((n = file1.read(buf, sizeof(buf))) > 0) {
    Serial.println(n);
    file2.write(buf, n);
  }
  file2.close();
}

void DeleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s/%s\n", fs, path);
  if (fs.remove(path)) Serial.println(F("File deleted"));
  else                 Serial.println(F("Delete failed"));
}

//
// HTML pages with buttons & inputs management
//____________________________________________________________________________________________

// actions done from the named_html[] as long as there are %PLACEHOLDER% keys.
// These palceholders will then be replaced with the actual HTML text to build the page once loaded.
// Replaces placeholder with button section in your web page
String processor(const String& var) {

  String h = "";

// main webpage
  if(var == "RADIOPLACEHOLDER") {
    h = "<p><h3>station de eRadio: " + String(stations[radioNum]) + "</h3><p>";
    h += "<p><a href='/stationprec'><button class='button button3'><</button></a>";
    h += "<a href='/stationsuiv'><button class='button button3'>></button></a></p>";
    h += "<HR size=2 align=center>";

    h += "<p><h3>supprimer la station de eRadio: " + String(radioNum +1) +"</h3><p>";
    h += "<p><a href='/confirmdelete'><button class=\"button\">Supprimer station</button></a></p>";

    h += "<p><h3>ajouter une station, saisir l'url:</h3><p>";
    h += "<form action='/addradio'><input type='text' name='r'>";
    h += "<INPUT class='button button3' type='submit' value='Valider'></form></p>";

    h += "<HR size=2 align=center>";
    h += "<p><h3>Radio fr&eacute;quence: " + String(rfFreq) + "kHz</h3><p>";
    h += "<p><a href='/freqmoins'><button class='button button3'>-</button></a>";
    h += "<a href='/freqplus'><button class='button button3'>+</button></a></p>";

    h += "<p><h3>Taux de modulation: " + String(map(volume, 0, 21, 0, 100)) + "%</h3><p>";
    h += "<p><a href='/modulmoins'><button class='button button3'>-</button></a>";
    h += "<a href='/modulplus'><button class='button button3'>+</button></a></p>";

    h += "<HR size=2 align=center>";
    h += "<p><h3>rotation &eacute;cran:</h3><p>";
    h += "<p><a href='/rotation'><button class='button button3'>@</button></a></p>";

    h += "<p><h3>" + VERSION + "</h3></p></BODY></center></html>";
    return h;
  }

// confirm delete radio station
  if(var == "CONFIRMDELETEPLACEHOLDER") {
    h = "<p><h3>confirmer la suppression de la station de eRadio: " + radioName +"</h3><p>";
    h += "<p><a href='/deleteradio'><button class='button'>Oui</button></a>";
    h += "<a href='/radio'><button class='button button2'>Non</button></a></p>";
    return h;
  }

  return String();
}      // end of processor()

Here is a device to make old radios alive again!

<img width="880" height="542" alt="AM transmitter diagram" src="https://github.com/user-attachments/assets/25f5cf2b-6bff-43a5-8cb5-dba886002823" />

I wanted a very simple device, low energy, easy to use, that will AM transmit so that I can listen back my old radios.

Here the signal to transmit comes from a builtin web-radio, instead of a Bluetooth receiver.

Web radio stations are received by wifi, then the "analog" audio signal is PWMed at a chosen radio frequency.

The analog audio is 8 bits defined; but it is enough for a last century radio player!

Due to the PWM characteristics of the ESP32, for a 8 bits definition, the PWM frequency must be under 300kHz.

So the transmitter looks like more for a LW reception than  MW. Nevertheless, the output is a square signal, so the harmonic 2 and 3  can be listened on MW.
The power output is so ridiculous with a power supply of 3.3V that you will not disturb your neighbors.

The antenna can be a simple wire, or an old antenna frame  like the one I use. The distance between the antenna frame and the radio  is less than 1 meter.

The TTGO T-Display offers the connection for a Lithium battery, so the transmitter is a portable device.
Take care of the IDE, Expressif, and Audio library version.

![20251026_174141](https://github.com/user-attachments/assets/492d1371-a821-48fd-be48-b808be0cd142)
![20251026_174134](https://github.com/user-attachments/assets/32bac4fa-da4a-4db5-8ed6-d6e133c85555)
![20251026_174154](https://github.com/user-attachments/assets/2f2d9e73-a5f1-486c-8b44-c53b7b297f54)




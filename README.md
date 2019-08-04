TinyTX4_BME280
--------------

This Arduino IDE sketch is for a TinyTX4 Tranceiver from meigrafd to read and transmit the BME280 I2C humidity, pressure and temperature sensor data to the TinyRX4 Receiver.

Based on Sketch from meigrafd: https://github.com/meigrafd/TinyTX4

TinyTX4 / TinyRX4 meigrafd Project-Page: https://forum-raspberrypi.de/forum/thread/7472-batteriebetriebene-funk-sensoren/

Thanks to technoblogy for the very tiny 'tiny-bme280' library, which I have implemented in the sketch: https://github.com/technoblogy/tiny-bme280

### Make sure that your BME280-Board supports 5V power input. Some boards, such as Adafruit's, support either 5V or 3.3V operation, so check before.
### Don't forget the pull-up resistors (4,7KOhm) on SCL and SDA.
### The expected default I2C-slave address is 0x76. You can change it in the sketch (BME280address) if necessary.

## Compiling and programming the Attiny84 with Arduino IDE

The following librarys are needed to compile the sketch:

- RFM12B : https://github.com/LowPowerLab/RFM12B/archive/master.zip

- SoftI2CMaster: https://github.com/felias-fogg/SoftI2CMaster


I compiled the sketch in following enviroment:

- Arduinon IDE Version 1.8.5

- ATTiny Core from SpenceKonde: https://github.com/SpenceKonde/ATTinyCore with following configurations:

    - Board: ATtiny24/44/84
    - Pin Mapping: Counterclockwise (like old ATTinyCore and x41-Series)
    - Chip: ATtiny84
    - Clock: 8 MHz (internal)
    - EEPROM: EEPROM retained
    - LTO (1.6.11+ only): Enabled
    - B.O.D. Level: B.O.D. Disabled
	- Programmer: Arduino as ISP (ATTiny Core)

- Arduino Nano V3.0 clone as ISP

	- Upload the Sketch 'ArduinoISP' from examples to the Arduino
    - set Board to ATiny84 Hardware as described above
    - Burn Bootloader on the ATtiny84
    - Upload sketch to Attiny84

If you run in to an error like: 'avrdude: stk500_getsync(): not in sync: resp=0x15' already at burning bootloader.  
Try an 10µF Capacitor between Arduino RESET and GND pin while programming the ATtiny84.


## ATtiny84 PIN Configuration 
SDA, SCL, LED and SensorPower pin can be changed in the sketch if necessary

                               +-\/-+
    VCC                  VCC  1|    |14  GND           GND
    ---             (D0) PB0  2|    |13  AREF (D10)    SensorPower(BME280)
    nSEL(RFM12B)    (D1) PB1  3|    |12  PA1 (D9)      LED
    ---                RESET  4|    |11  PA2 (D8)      SCL(BME280)
    nIRQ(RFM12B)    (D2) PB2  5|    |10  PA3 (D7)      SDA(BME280)
    ---             (D3) PA7  6|    |9   PA4 (D6)      SCK(RFM12B)
    SDO(RFM12B)     (D4) PA6  7|    |8   PA5 (D5)      SDI(RFM12B)
                               +----+



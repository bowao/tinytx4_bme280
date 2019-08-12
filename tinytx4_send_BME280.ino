

// RFM12B Sender for BME280 I2C humidity, pressure and temperature sensor
//
// Basiert zum Teil auf der Arbeit von Nathan Chantrell
//
// modified by meigrafd @ 11.12.2014
//
// modified for BME280 by bowao @ 03.08.2019
// Needs the 'SoftI2CMaster' library from felias-fogg: https://github.com/felias-fogg/SoftI2CMaster
// Thanks to technoblogy, for the very tiny 'tiny-bme280' library, which I have implemented here.
// https://github.com/technoblogy/tiny-bme280
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// I2C PIN-Konfiguration ----> Don't forget the pull-up resistors (4.7KOhm)
//------------------------------------------------------------------------------
//SDA pin is connected on D7 (ATtiny Pin 10) (PA3)
#define SDA_PORT PORTA
#define SDA_PIN 3

//SCL pin is connected on D6 (ATtiny Pin 11) (PA2)
#define SCL_PORT PORTA
#define SCL_PIN 2
//------------------------------------------------------------------------------

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <SoftWire.h>
SoftWire Wire = SoftWire();

#include <RFM12B.h>

// Power-Save-Stuff.
// http://www.surprisingedge.com/low-power-atmegatiny-with-watchdog-timer/
// https://www.sparkfun.com/tutorials/309
// http://jeelabs.org/tag/lowpower/
#include <avr/sleep.h>
volatile int watchdog_counter;

// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) { watchdog_counter++; }

//------------------------------------------------------------------------------
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID         10   // network ID used for this unit
#define NETWORKID     210   // the network ID we are on
#define GATEWAYID      22   // the node ID we're sending to
#define ACK_TIME     2000   // # of ms to wait for an ack
#define requestACK   false   // request ACK? (true/false)

//------------------------------------------------------------------------------
// PIN-Konfiguration
//------------------------------------------------------------------------------
// BME280 Power pin is connected on D10 (ATtiny pin 13)
#define BME280_POWER 10      

// LED pin is connected on D9 (ATtiny pin 12)
#define LEDpin 9            
//------------------------------------------------------------------------------
/*
                               +-\/-+
    ---                  VCC  1|    |14  GND           ---
    ---             (D0) PB0  2|    |13  AREF (D10)    POW
    nSEL            (D1) PB1  3|    |12  PA1 (D9)      LED
    ---                RESET  4|    |11  PA2 (D8)      SCL
    nIRQ  INT0  PWM (D2) PB2  5|    |10  PA3 (D7)      SDA
    ---         PWM (D3) PA7  6|    |9   PA4 (D6)      SCK
    SDO         PWM (D4) PA6  7|    |8   PA5 (D5) PWM  SDI
                               +----+
*/

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
//#define KEY   "a4gBM69UZ03lQyK4"

// Need an instance of the Radio Module
RFM12B radio;

// Temperatur-String zum Versand per 433 Mhz
char msg[34];

//############################################################################
//####################### tiny-bme280 by technoblogy #########################

int16_t T[4], P[10], H[7];
int32_t BME280t_fine;

int BME280address = 118;                      // I2C-Slave Address (118 = 0x76)

int16_t read16 () {
  uint8_t lo, hi;
  lo = Wire.read(); hi = Wire.read();
  return hi<<8 | lo;
}

int32_t read32 () {
  uint8_t msb, lsb, xlsb;
  msb = Wire.read(); lsb = Wire.read(); xlsb = Wire.read();
  return (uint32_t)msb<<12 | (uint32_t)lsb<<4 | (xlsb>>4 & 0x0F);
}

void BME280setup () {
  Wire.begin();
  delay(2);
  // Set the mode to Forced, x1 upsampling
  Wire.beginTransmission(BME280address);
  Wire.write(0xF2);                             // ctrl_hum
  Wire.write(0b001);                            // Humidity oversampling x1
  Wire.write(0xF4);                             // ctrl_meas
  Wire.write(0b00100101);                       // Temperatur and Pressure oversampling x1, Forced mode
  delay(10);                                    // Allow 10ms for the sensor to measure (depends on oversampling)
  // Read the chip calibrations.
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 26);
  for (int i=1; i<=3; i++) T[i] = read16();     // Temperature
  for (int i=1; i<=9; i++) P[i] = read16();     // Pressure
  Wire.read();  // Skip 0xA0
  H[1] = (uint8_t)Wire.read();                  // Humidity
  //
  Wire.beginTransmission(BME280address);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 7);
  H[2] = read16();
  H[3] = (uint8_t)Wire.read();
  uint8_t e4 = Wire.read(); uint8_t e5 = Wire.read();
  H[4] = ((int16_t)((e4 << 4) + (e5 & 0x0F)));
  H[5] = ((int16_t)((Wire.read() << 4) + ((e5 >> 4) & 0x0F)));
  H[6] = ((int8_t)Wire.read()); // 0xE7
  // Read the temperature to set BME280t_fine
  BME280temperature();
}

// Trigger new forced measurement, cause sensor is in forced mode (see BME280setup)
// Not in use here, cause on every power-on we call BME280setup
// Maybe Helpful for other purpose 
void BME280remeasure() {
  Wire.beginTransmission(BME280address);
  Wire.write(0xF4);                             // ctrl_meas
  Wire.write(0b00100101);                       // Temperatur and Pressure oversampling x1, Forced mode
  Wire.endTransmission();
  delay(10);                                    // Allow 10ms for the sensor to measure (depends on oversampling)
}

// Returns temperature in DegC, resolution is 0.01 DegC
// Output value of “5123” equals 51.23 DegC
int32_t BME280temperature () {
  Wire.beginTransmission(BME280address);
  Wire.write(0xFA);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 3);
  int32_t adc = read32();
  // Compensate
  int32_t var1, var2; 
  var1 = ((((adc>>3) - ((int32_t)((uint16_t)T[1])<<1))) * ((int32_t)T[2])) >> 11;
  var2 = ((((adc>>4) - ((int32_t)((uint16_t)T[1]))) * ((adc>>4) - ((int32_t)((uint16_t)T[1])))) >> 12);
  var2 = (var2 * ((int32_t)T[3])) >> 14;
  BME280t_fine = var1 + var2;
  return (BME280t_fine*5+128)>>8;
}

// Returns pressure in Pa as unsigned 32 bit integer
// Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t BME280pressure () {
  Wire.beginTransmission(BME280address);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 3);
  int32_t adc = read32();
  // Compensate
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)BME280t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)P[6]);
  var2 = var2 + ((var1*((int32_t)P[5]))<<1);
  var2 = (var2>>2) + (((int32_t)P[4])<<16);
  var1 = (((P[3] * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)P[2]) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((int32_t)((uint16_t)P[1])))>>15);
  if (var1 == 0) return 0;
  p = (((uint32_t)(((int32_t)1048576) - adc) - (var2>>12)))*3125;
  if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
  else p = (p / (uint32_t)var1) * 2;
  var1 = (((int32_t)P[9]) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)P[8]))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + P[7]) >> 4));
  return p;
}

// Humidity in %RH, resolution is 0.01%RH
// Output value of “4653” represents 46.53 %RH
uint32_t BME280humidity () {
  Wire.beginTransmission(BME280address);
  Wire.write(0xFD);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 2);
  uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
  int32_t adc = (uint16_t)(hi<<8 | lo);
  // Compensate
  int32_t var1; 
  var1 = (BME280t_fine - ((int32_t)76800));
  var1 = (((((adc << 14) - (((int32_t)H[4]) << 20) - (((int32_t)H[5]) * var1)) +
  ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)H[6])) >> 10) * (((var1 *
  ((int32_t)H[3])) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
  ((int32_t)H[2]) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)H[1])) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  return (uint32_t)((var1>>12)*25)>>8;
}

//######################################################################################
//######################################################################################

static void activityLed (byte state, byte time = 0) {
  if (LEDpin) {
    pinMode(LEDpin, OUTPUT);
    if (time == 0) {
      digitalWrite(LEDpin, state);
    } else {
      digitalWrite(LEDpin, state);
      delay(time);
      digitalWrite(LEDpin, !state);
    }
  }
}

// blink led
static void blink (byte pin, byte n = 3) {
  if (LEDpin) {
    pinMode(pin, OUTPUT);
    for (byte i = 0; i < 2 * n; ++i) {
      delay(100);
      digitalWrite(pin, !digitalRead(pin));
    }
  }
}

void enableADC(bool b) {
  if (b == true){
    bitClear(PRR, PRADC); // power up the ADC
    ADCSRA |= bit(ADEN); // enable the ADC
    delay(10);
  } else {
    //ADCSRA &= ~(1<<ADEN); // Disable ADC, saves ~230uA
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    bitSet(PRR, PRADC); // power down the ADC
  }
}

void goToSleep() {
  // SLEEP_MODE_IDLE -the least power savings
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN -the most power savings
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode.
  sleep_enable(); // Enable sleep mode.
  sleep_mode(); // Enter sleep mode.

  // After waking from watchdog interrupt the code continues to execute from this point.

  sleep_disable(); // Disable sleep mode after waking.
  // Re-enable the peripherals.
  //power_all_enable();
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage (in mV)
//--------------------------------------------------------------------------------------------------
long readVcc() {
  enableADC(true);
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  //result = 1125300L / result; // Back-Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = 1126400L / result; // Back-calculate Vcc in mV
  enableADC(false);
  return result;
}

// init Setup
void setup() {
  pinMode(BME280_POWER, OUTPUT);  // set power pin for Sensor to output

  // configure RFM12B
  radio.Initialize(NODEID, RF12_433MHZ, NETWORKID);
  #ifdef KEY
    radio.Encrypt((byte*)KEY);      //comment this out to disable encryption
  #endif
  radio.Control(0xC040);   // Adjust low battery voltage to 2.2V
  radio.Sleep(); //sleep right away to save power

  watchdog_counter = 500; //set to have an initial transmition when starting the sender.

  enableADC(false); // power down/disable the ADC
  //ACSR = (1<<ACD); // Disable the analog comparator
  //DIDR0 = 0x3F; // Disable digital input buffers on all ADC0-ADC5 pins.
  //ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
  PRR = bit(PRTIM1); // only keep timer 0 going

  setup_watchdog(9); //Wake up after 8 sec

  analogReference(INTERNAL);   // Set the aref to the internal 1.1V reference
  if (LEDpin) {
    activityLed(1,1000); // LED on
  }
}

// Loop
void loop() {
  goToSleep(); // goes to sleep for about 8 seconds and continues to execute code when it wakes up

  // 15 * 4 sec = 1 Min , 75 * 4 sec = 5 Min , 225 * 4 sec = 15 min
  // 7,5 * 8sec = 1 Min. / 37,5 * 8sec = 5Min. / 22,5 * 8sec = 3min. / 15 * 8sec = 2min.
  if (watchdog_counter >= 22) {
    watchdog_counter = 1;

    //activityLed(1);  // LED on
    pinMode(BME280_POWER, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(BME280_POWER, HIGH); // turn Sensor on
    delay(50); // Allow 50ms for the sensor to be ready
    enableADC(true); // power up/enable the ADC

    BME280setup();
    
    int vcc  = readVcc(); // Get supply voltage
    int temp = BME280temperature();
    int hum = BME280humidity();
    int32_t pres = BME280pressure();
    
      // msg-Variable mit Daten zum Versand fuellen, die spaeter an das WebScript uebergeben werden
      //snprintf(msg, 26, "v=%d&t=%d&h=%d", vcc, temp, humi);  // uses too much memory
      strcpy(msg,"v=");
      itoa(vcc,&msg[strlen(msg)],10);
      strcat(msg,"&t=");
      itoa(temp,&msg[strlen(msg)],10);
      strcat(msg,"&h=");
      itoa(hum,&msg[strlen(msg)],10);
      strcat(msg,"&p=");
      ltoa(pres,&msg[strlen(msg)],10);
      
      radio.Wakeup();
      radio.Send(GATEWAYID, (uint8_t *)msg, strlen(msg), requestACK);
      radio.SendWait(2);    //wait for RF to finish sending (2=standby mode, 3=power down)
      radio.Sleep();
  
      if (LEDpin) {
        blink(LEDpin, 2); // blink LED
      }
    //--}
    digitalWrite(BME280_POWER, LOW); // turn Sensor off to save power
    pinMode(BME280_POWER, INPUT); // set power pin for Sensor to input before sleeping, saves power
    enableADC(false); // power down/disable the ADC
  }

}

// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
// From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Correct incoming amount if need be
  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watchdog reset
  WDTCSR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCSR = bb; //Set new watchdog timeout value
  WDTCSR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}


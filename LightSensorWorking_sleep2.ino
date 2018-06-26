/* This is a program for a light sensor that sleeps until a light intesity over an arbitray threshold is detected by the sensor.
 *  The sensor records the timestamp, the lux value, and logs them on an SD card.
 *  When there is no light the sensor sleeps.
 */

#include <avr/sleep.h>          //You can try the lowpower.h library here but this one worked for me
#include <avr/interrupt.h>

#include <SD.h>                 // I much prefer SdFat.h by Greiman over the old SD.h library used here
#include  <SPI.h>
const int chipSelect = 10;      //CS moved to pin 10 on the arduino
#include "LowPower.h"
#include <RTClib.h>

RTC_DS3231 RTC;
// creates an RTC object in the code
// variables for reading the RTC time & handling the INT(0) interrupt it generates
#define DS3231_I2C_ADDRESS 0x68
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char CycleTimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds)

const char codebuild[] PROGMEM = __FILE__;  // loads the compiled source code directory & filename into a varaible
const char header[] PROGMEM = "Timestamp, Lux"; //gets written to second line datalog.txt in setup

//indicator LED pins - change to suit your connections
int RED_PIN = 7;
int GREEN_PIN = 6;
int BLUE_PIN = 5;
//============================================================================================================================
//Below taken from Adafruit TSL2561 example code

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
/*
   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc.
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.

*/
//MUST add false tag or else sensor will default to sleep mode between readings
//Interrupts won't work without false being declared

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345, false);

// We assume the Adafruit TSL2561's "int" pin is attached to this digital pin
#define INTERRUPT_PIN 3

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
   tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  //tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
   tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Configure interrupt thresholds */
  // First: convert lux value to raw sensor value, using "sunlight" approximation.
  // Other approximations, see Adafruit_TSL2561_U.h
  uint32_t threshold = tsl.calculateRawCH0(3000, TSL2561_APPROXCHRATIO_SUN);
  //Serial.println("threshold: "); Serial.println(threshold);
  tsl.setInterruptThreshold(0,10); //High value is channel 0 data

  /*
        Maximum value is 15; put to 1 for immediate triggering; put to 0 to have
        the interrupt triggered after every integration time (regardless of whether
        the thresholds were exceeded or not)
  */
  tsl.setInterruptControl(TSL2561_INTERRUPTCTL_LEVEL, 1);
  tsl.clearLevelInterrupt();

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("16x");
  Serial.print  ("Timing:       "); Serial.println("101 ms");
  Serial.println("------------------------------------");
}
//Above taken from Adafruit TSL2561 Example Code
//===========================================================================================

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Light Sensor Test"); Serial.println("");

  /* Configure the interrupt pin as input pin */
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  /* Initialise the sensor */
  //use tsl.begin() to default to Wire,
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  /* We're ready to go! */
  Serial.println("");

//========================================================================================
//                            SD Card Setup
//----------------------------------------------------------------------------------------
// Setting the SPI pins high helps some sd cards go into sleep mode
// the following pullup resistors only needs to be enabled for the ProMini builds - not the UNO loggers
pinMode(chipSelect, OUTPUT); digitalWrite(chipSelect, HIGH); //ALWAYS pullup the ChipSelect pin with the SD library
//and you may need to pullup MOSI/MISO
pinMode(11, OUTPUT); digitalWrite(11, HIGH); //pullup the MOSIpin=11
pinMode(12, INPUT); digitalWrite(12, HIGH);  //pullup the MISOpin=12
delay(1);

Wire.begin();          // start the i2c interface for the RTC
RTC.begin();           // start the RTC
while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
}
Serial.print(F("Initializing SD card…"));
// print lines in the setup loop only happen once
// see if the card is present and can be initialized
if (!SD.begin(chipSelect)) {
  Serial.println(F("Card failed, or not present"));
  // don’t do anything more:
  return;
}
Serial.println(F("card initialized."));

// You must already have a plain text file file named ‘datalog.txt’ on the SD already for this to work!

//————-print a header to the data file———- OPTIONAL!
File dataFile = SD.open("datalog.txt", FILE_WRITE);
if (dataFile) { // if the file is available, write to it:
  dataFile.println((__FlashStringHelper*)codebuild); // writes the entire path + filename to the start of the data file
  dataFile.println((__FlashStringHelper*)header);
  dataFile.close();
}
else {
   Serial.println("error opening datalog.txt"); // if the file isn’t open, pop up an error:
}

pinMode(RED_PIN, OUTPUT); //configure 3 RGB pins as outputs, not used beyond startup
pinMode(GREEN_PIN, OUTPUT);
pinMode(BLUE_PIN, OUTPUT);
digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, HIGH); // startup with green led lit
digitalWrite(BLUE_PIN, LOW);
}

//============================ACTIVATES SLEEP======================================================
void enterSleep() {
  delay(500);            //Without the delay the Arduino will sleep before intitializing SD Card
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), lightInterrupt, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     //AVR sleep library, most power-saving option
  sleep_enable();
  sleep_mode();

  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
}

//=====================MAIN LOOP====================================================================
void loop(void)
{
String dataString = "";                  //Creates empty string named dataString
enterSleep();                            //Code starts after wherever this is places
  /* Interrupt triggered? */
  if(digitalRead(INTERRUPT_PIN) == LOW) {  //Could possibly be replaced with lightInterrupt == true, Schroedinger's interrupt right now
    // We have an interrupt!
    Serial.println("TSL2561 reported interrupt thresholds exceeded!");

//------------------------------------------------------------------------------------------------------------------------
    DateTime now = RTC.now(); //this reads the time from the RTC
    sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
    //loads the time into a string variable

//-------------------------------------------------------------------------------------------------------------------------
//polls(?) sensor and if light is above threshold specified prints it to serial monitor
    sensors_event_t event;
    tsl.getEvent (&event);
    if (event.light) {
      Serial.print(event.light); Serial.println (" lux");
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
//-------------------------------------------------------------------------------------------------------------------------
//concatenates timestamp and lux data to dataString and writes to SD card

     // dataString += ("Alarm triggered at: ");
      dataString += CycleTimeStamp;
      dataString += (", ");
     // dataString += ("lux = ");
      dataString += event.light;
    }
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("error opening datalog.txt"));
  }
//-------------------------------------------------------------------------------------------------------------------------
    // Clear interrupt on sensor
    tsl.clearLevelInterrupt();
  } //End of interrupt
} //end of main loop

//============================LIGHT ISR============================================================

void lightInterrupt() {
  digitalRead(INTERRUPT_PIN) == LOW;
}

// EEP 523 A5/A6
// To develop this code below for the Arduino side, the student developer 
// referred to the two circuit playground example, the hello_button and HRM from class

#include <Adafruit_Circuit_Playground.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//For temperature measurement:
#include <Adafruit_CircuitPlayground.h>

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


#define echoPin 6 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 12 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

// The Circuit Playground device will send over the thermo data 
// through the uart interface 
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// Define ID varibles for BLE service
int32_t thermoServiceId;
int32_t MeasureCharId;
int32_t thermoLocationCharId;

// bool to track whether buttons are pressed
bool leftButtonPressed;
bool rightButtonPressed;


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;

  // setup baudrate 
  Serial.begin(115200);
  Serial.println(F("Rider Safety Device"));
  Serial.println(F("---------------------------------------------------"));

  CircuitPlayground.begin();

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Rider Safety Device': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Rider Safety Device")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Thermo Clicker Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Thermo Clicker Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &thermoServiceId);
  if (! success) {
    error(F("Could not add Thermo Clicker service"));
  }

  /* Add the Thermo Clicker Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Thermo Clicker Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &MeasureCharId);
    if (! success) {
    error(F("Could not add Thermo Clicker characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &thermoLocationCharId);
    if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Thermo Clicker Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();
}

/** Send randomized Thermo Clicker data continuously **/
void loop(void)
{

  // store button press into boolean varible
  leftButtonPressed = CircuitPlayground.leftButton();
  rightButtonPressed = CircuitPlayground.rightButton();
  int temperature_measure = 0;
  measure_distance();

  Serial.print(F("Updating Rider Safety Device value to "));
  Serial.print(distance);
  Serial.println(F("Rider Safety Device"));

  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( MeasureCharId );
  ble.print( F(",00-") );
  ble.println(distance, HEX);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }

  /* Delay before next measurement update */
  delay(500);
}

void measure_distance() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
  Serial.print("Distance: ");
  if (distance > 255) {
    distance = 255;
  }
  Serial.print(distance);
  Serial.println(" cm");
}


#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <stdlib.h>
#include <utility/imumaths.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 3     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("NEWNAME"); /* 7 characters max! */

  BTLEserial.begin();

//    if(!bmp.begin())
  //{
    /* There was a problem detecting the BMP085 ... check your connections */
    //Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  //}
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }
  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
    
   sensors_event_t event;
  bno.getEvent(&event);
    char tempCharX[10];
    String tempStringX="";
    dtostrf(event.pressure.x, 5, 3, tempCharX);
    for(int i = 0; i<sizeof(tempCharX); i++)
    {
      tempStringX+=tempCharX[i];
    }
    Serial.println(tempStringX);

     char tempCharY[10];
    String tempStringY="";
    dtostrf(event.pressure.y, 5, 3, tempCharY);
    for(int i = 0; i<sizeof(tempCharY); i++)
    {
      tempStringY+=tempCharY[i];
    }
    Serial.println(tempStringY);
     
     char tempCharZ[10];
    String tempStringZ="";
    dtostrf(event.pressure.z, 5, 3, tempCharZ);
    for(int i = 0; i<sizeof(tempCharZ); i++)
    {
      tempStringZ+=tempCharZ[i];
    }
    Serial.println(tempStringZ);

    String tempString="X:" + tempStringX + "Y:" + tempStringY + "Z: "+ tempStringZ;
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = tempString;//Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }

 
  /* Display the results (barometric pressure is measure in hPa)*/ 
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa*/ 

    //Serial.print("Pressure: "); Serial.print(event.pressure); Serial.println(" hPa");
  }
  else
  {
    Serial.println("Sensor error");
  } 
  delay(700);
}

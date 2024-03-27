/*
   This sketch can be used to calibrate your RC to be mapped from -1 to 1. The calibration is then saved
   in the EEPROM to be used in a different sketch withyout the need of burning the calibration in your code.

   To use different ports as the one i used, just type your ports in under "channelPins". The total amount
   of pins you are using is to be defined under "CHANNELAMOUNT".

https://github.com/timoxd7/FastRCReader/blob/master/examples/Calibration_Mapping_to_EEPROM/Calibration_Mapping_to_EEPROM.ino
*/

#include <Arduino.h>
#include "FastRCReader.h"
#include <EEPROM.h>

#define _ON  HIGH
#define _OFF LOW

//Port definitions is set in FastRCReader.h
//Change with the Ports you want to use
#define CHANNELAMOUNT 2
const uint8_t channelPins[CHANNELAMOUNT] = {3, 5};

const int potPin = A0;
int potVal = -1;

//The adress the calibration/mapping will be saved in the EEPROM
#define EEPROMADRESS 0

////RCChannelMapper RC;
FastRCReader  RC;


//Fuction which should wait until the next Calibration point is triggeret by serial
void waitSerial() {
  while (true) {
    while (Serial.available()) {
      char got = Serial.read();
      if (got == 'N' || got == 'n') return;
    }
  }
}

//==========================================================================================
void setup() {
  pinMode(2, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(2, _OFF);
  digitalWrite(13, _OFF);
  Serial.begin(115200);

  delay(500);
  //while (!Serial.available());

  digitalWrite(2, _ON);
  RC.begin();

  for (uint8_t i = 0; i < CHANNELAMOUNT; i++) {     //instantiate RC channel as interrupt at pin set in channelPin array
    RC.addChannel(channelPins[i]);
  }

  //Get the mins and the maxs of all channels to map them
  for (uint8_t i = 0; i < CHANNELAMOUNT; i++) {
    //Write what is to do in Serial Monitor
    char buffer[57];
    sprintf(buffer, "Put Channel %i on Pin %i to Min position, then type n + ENTER ", i + 1, channelPins[i]);
    Serial.print(buffer);

    //Then save Min value
    waitSerial();
    uint16_t channelMin = RC.getFreq(channelPins[i]);
    Serial.println(channelMin);

    //Same again
    sprintf(buffer, "Put Channel %i on Pin %i to Max position, then type n + ENTER ", i + 1, channelPins[i]);
    Serial.print(buffer);

    waitSerial();
    uint16_t channelMax = RC.getFreq(channelPins[i]);
    Serial.println(channelMax);
  }

  digitalWrite(13, _ON);
  
  //Save to EEPROM
  EEPROM.put(EEPROMADRESS, RC);
  Serial.println("\nSaved in EEPROM, you can now run your own code by reading the RC-Object from the EEPROM");

  delay(1000);

  while(1) {
    potVal = analogRead(potPin);   //read pot
    Serial.println(potVal);
    delay(500);
  } 
}


//**************************************************************************************
void loop() {
    uint16_t channelVal0 = RC.getFreq(channelPins[0]);
    uint16_t channelVal1 = RC.getFreq(channelPins[1]);
    Serial.print(channelVal0); Serial.print(" "); Serial.println(channelVal1);
    delay(500);
}
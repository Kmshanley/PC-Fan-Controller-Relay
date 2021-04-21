#include <Arduino.h>
#include <PID_v1.h>
#include <U8g2lib.h>
#include <Wire.h>

//-------PIN Mapping------------
const uint8_t encoderPin1 = PA8;
const uint8_t encoderPin2 = PA9;
const uint8_t encoderSwitchPin = PA10; //Encoder Press Switch

const uint8_t Fan1OutPin = PA2;
const uint8_t Fan2OutPin = PA3;
const uint8_t Fan1InPin = PA4;
const uint8_t Fan2InPin = PA5; 

const uint8_t temp1Pin = PB1;
const uint8_t temp2Pin = PB0;

//--------------------------------

void updateEncoder();
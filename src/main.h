#include <Arduino.h>
#include "QuadEncoder.h"
#include <FreqMeasureMulti.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ADC.h>
#include <ADC_util.h>
#include <thermistor.h>



const uint8_t knobOnePin1 = 0;
const uint8_t knobOnePin2 = 0; 
const uint8_t knobOnePinPush = 0;

const uint8_t knobTwoPin1 = 0; 
const uint8_t knobTwoPin2 = 0;
const uint8_t knobTwoPinPush = 0;

const uint8_t fan1ControlPin = 2; 
const uint8_t fan2ControlPin = 3;
const uint8_t pump1ControlPin = 4;

const uint8_t fan1InputTachPin = 5; 
const uint8_t fan2InputTachPin = 6; 
const uint8_t pump1InputTachPin = 7; 

const uint8_t temp1InputPin = 23; 
const uint8_t temp2InputPin = 22;
const uint8_t temp3InputPin = 21;

const uint8_t onewireBus = 41;

const int fixedResistor = 5100;

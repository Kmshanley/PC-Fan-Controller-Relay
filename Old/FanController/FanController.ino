#include <PID_v1.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <itoa.h>
#include "quadratureBluePill.h"
#include <HardwareTimer.h>


//-------PIN Mapping------------
const uint8_t encoderPin1 = PA8;
const uint8_t encoderPin2 = PA9;
const uint8_t encoderSwitchPin = PA10; //Encoder Press Switch

const uint8_t Fan1OutPin = PA7;
const uint8_t Fan2OutPin = PA3;
const uint8_t Fan1InPin = PA6;
const uint8_t Fan2InPin = PA5;

const uint8_t temp1Pin = PB1;
const uint8_t temp2Pin = PB0;

const uint8_t Switch1Pin = PB4;
const uint8_t Switch2Pin = PB5;

//--------------Constants------------------

#define BCOEFFICIENT 3435
#define TEMPERATURENOMINAL 25
#define THERMISTORNOMINAL 10000
#define SERIESRESISTOR 10000

#define NUMSAMPLES 5


const int Fan1MinRPM = 450;
const int Fan1MaxRPM = 1500;


const int Fan2MinRPM = 400;
const int Fan2MaxRPM = 2400;


//------------

int knobVal;
unsigned int lastEncOut;
unsigned long debounceTimer;

volatile uint32_t rpm1Raw;
volatile uint32_t rpm2Raw;

uint32_t fan1Rpm;
uint32_t fan2Rpm;

bool fan1Toggle;
bool fan2Toggle;

int modPct = 100;

double F1Setpoint, F1Input, F1Output;
double F1Kp = 2, F1Ki = 5, F1Kd = 1;
PID Fan1PID(&F1Input, &F1Output, &F1Setpoint, F1Kp, F1Ki, F1Kd, DIRECT);

double F2Setpoint, F2Input, F2Output;
double F2Kp = 2, F2Ki = 5, F2Kd = 1;
PID Fan2PID(&F2Input, &F2Output, &F2Setpoint, F2Kp, F2Ki, F2Kd, DIRECT);

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

quadCounter  QC1(QUAD_TIMER_1);
HardwareTimer Tim3(3);

uint8_t pwmChannel1 = 1;
uint8_t pwmChannel2 = 2;

uint32_t rpmUpdateTimer1;
uint32_t statusTimer;


void setup() {
  Serial2.begin(115200);


  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(Switch1Pin, INPUT_PULLUP);
  pinMode(Switch2Pin, INPUT_PULLUP);
  pinMode(encoderSwitchPin, INPUT_PULLUP);
  pinMode(Fan1InPin, INPUT_PULLUP);
  pinMode(Fan2InPin, INPUT_PULLUP);

  pinMode(Fan1OutPin, PWM);
  pinMode(Fan2OutPin, PWM);

  Fan1PID.SetMode(AUTOMATIC);
  Fan2PID.SetMode(AUTOMATIC);

  //attachInterrupt(Switch1Pin, switchUpdate, CHANGE);
  //attachInterrupt(Switch2Pin, switchUpdate, CHANGE);

  attachInterrupt(Fan1InPin, rpm1Update, RISING);
  attachInterrupt(Fan2InPin, rpm2Update, RISING);



  display.begin();
  display.enableUTF8Print();

  Tim3.pause();
  Tim3.setPeriod(40);
  Tim3.refresh();
  Tim3.resume();

}


void loop() {
  switchUpdate();
  EncoderDebounce();
  if (millis() - rpmUpdateTimer1 > 1000) {
    fan1Rpm = rpm1Raw;
    fan2Rpm = rpm2Raw;
    rpm1Raw = 0;
    rpm2Raw = 0;
    rpmUpdateTimer1 = millis();
  }

  F1Input = fan1Rpm;
  F2Input = fan2Rpm;

  //float tyui = (getCTemp(temp1Pin) - 25) * (modPct / 100);
  float tyui = (40 - 25) * (modPct / 100);
  F1Setpoint = map(tyui, 0, 200, Fan1MinRPM, Fan1MaxRPM);
  F2Setpoint = map(tyui, 0, 200, Fan2MinRPM, Fan2MaxRPM);

  Fan1PID.Compute();
  Fan2PID.Compute();

  if (fan1Toggle) {
    pwmWrite(Fan1OutPin, 20000);
  }
  else {
    pwmWrite(Fan1OutPin, 0);
  }
  if (fan2Toggle) {
    pwmWrite(Fan2OutPin, F2Output);
  }
  else {
    pwmWrite(Fan2OutPin, 0);
  }


  if (millis() - statusTimer > 1000) {
    Serial2.print("Fan1: "); Serial2.print(fan1Toggle); Serial2.print(" : "); Serial2.println(fan1Rpm);
    Serial2.print("Fan2: "); Serial2.print(fan2Toggle); Serial2.print(" : "); Serial2.println(fan2Rpm);
    Serial2.print("Temp1: "); Serial2.print((int)getCTemp); Serial2.print(" : "); Serial2.println(modPct);
    Serial2.print("PID: "); Serial2.print(F1Output); Serial2.print(" : ");Serial2.println(F2Output);
    statusTimer = millis();
  }

}

float getCTemp (uint8_t pin) {
  uint32_t adcReading;
  for (uint8_t i; i < NUMSAMPLES; i++) {
    adcReading += analogRead(pin);
  }
  adcReading /= NUMSAMPLES;
  adcReading = 4095 / adcReading - 1;
  adcReading = SERIESRESISTOR / adcReading;

  float temp;
  temp = adcReading / THERMISTORNOMINAL;       // (R/Ro)
  temp = log(temp);                            // ln(R/Ro)
  temp /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  temp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  temp = 1.0 / temp;                           // Invert
  temp -= 273.15;                              // convert to C

  return temp;
}


void EncoderDebounce() {
  if (millis() - debounceTimer > 150) {
    unsigned int encOut = QC1.count();

    if (encOut > lastEncOut) {
      if (!(knobVal == -100)) {
        knobVal ++;
      }
      lastEncOut = encOut;
    }
    else if (encOut < lastEncOut) {
      if (!(knobVal == -100)) {
        knobVal --;
      }
      lastEncOut = encOut;
    }
    debounceTimer = millis();
  }
  
  modPct = map(knobVal, -100, 100, 0, 200);

}

void switchUpdate() {
  if (digitalRead(Switch1Pin) == LOW) {
    fan1Toggle = true;
  }
  else {
    fan1Toggle = false;
  }
  if (digitalRead(Switch2Pin) == LOW) {
    fan2Toggle = true;
  }
  else {
    fan2Toggle = false;
  }
}

void rpm1Update() {
  rpm1Raw ++;
}

void rpm2Update() {
  rpm2Raw ++;
}



void updateDisplay() {
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  display.drawStr(0, 10, "EncPos:: "); // write something to the internal memory
  display.sendBuffer();
  char cbuffer[20];
  //itoa(knobVal, cbuffer, 10)

  display.drawStr(20, 0, "10");

  display.sendBuffer();          // transfer internal memory to the display
}

#include <main.h>


//------------

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

double F1Setpoint, F1Input, F1Output;
double F1Kp=2, F1Ki=5, F1Kd=1;
PID Fan1PID(&F1Input, &F1Output, &F1Setpoint, F1Kp, F1Ki, F1Kd, DIRECT);

double F2Setpoint, F2Input, F2Output;
double F2Kp=2, F2Ki=5, F2Kd=1;
PID Fan2PID(&F2Input, &F2Output, &F2Setpoint, F2Kp, F2Ki, F2Kd, DIRECT);

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


void setup() {
  Serial.begin (9600);

  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);

  pinMode(encoderSwitchPin, INPUT);

  //turn pullup resistors on
  digitalWrite(encoderPin1, HIGH); 
  digitalWrite(encoderPin2, HIGH);
  digitalWrite(encoderSwitchPin, HIGH); 

  //call updateEncoder() when any high/low changed seen on encoder pins
  attachInterrupt(encoderPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderPin2, updateEncoder, CHANGE);

  Fan1PID.SetMode(AUTOMATIC);
  Fan2PID.SetMode(AUTOMATIC);

  display.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  

}

void loop(){ 
  display.clearBuffer();      
  display.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  display.drawStr(0,10,"Hello World!");  // write something to the internal memory
  
  display.sendBuffer();     

  Fan1PID.Compute();
  Fan2PID.Compute();

  analogRead()

}
/*
u8g2.clearBuffer();          // clear the internal memory
u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
u8g2.drawStr(0,10,"Hello World!");  // write something to the internal memory
u8g2.sendBuffer();          // transfer internal memory to the display
*/

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number 
  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++; 
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --; 
  lastEncoded = encoded; //store this value for next time 
}
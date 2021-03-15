#include <main.h>

QuadEncoder knobOne(1, knobOnePin1, knobOnePin2, 1); 
QuadEncoder knobTwo(2, knobTwoPin1, knobTwoPin2, 1); 

FreqMeasureMulti fan1freq;
FreqMeasureMulti fan2freq;



uint32_t knobOneValue;
uint32_t knobTwoValue;

double fan1Setpoint = 0;
double fan1Input, fan1Output;
double fan2Setpoint = 0; 
double fan2Input, fan2Output;

uint8_t knobMode;

//Specify the links and initial tuning parameters
double Kp=5, Ki=5, Kd=1;
PID fan1PID(&fan1Input, &fan1Output, &fan1Setpoint, Kp, Ki, Kd, REVERSE);
PID fan2PID(&fan2Input, &fan2Output, &fan2Setpoint, Kp, Ki, Kd, REVERSE);

OneWire oneWire(onewireBus);
DallasTemperature ambientTemp(&oneWire);

float airTemp;

elapsedMillis displayTimer;
elapsedMillis airTempTimer; 
elapsedMillis knobTimer;

void setup() {
  Serial.begin(9600);
  
  knobOne.setInitConfig();
  knobOne.EncConfig.INDEXTriggerMode = RISING_EDGE;
  knobTwo.setInitConfig();
  knobTwo.EncConfig.INDEXTriggerMode = RISING_EDGE;

  pinMode(fan2ControlPin, OUTPUT);
  pinMode(fan1ControlPin, OUTPUT);

  knobOne.init();
  knobTwo.init();

  fan2freq.begin(fan2InputTachPin);
  fan1freq.begin(fan1InputTachPin);

  fan1PID.SetMode(AUTOMATIC);
  fan2PID.SetMode(AUTOMATIC);

  fan1PID.SetOutputLimits(11, 4096);
  fan2PID.SetOutputLimits(11, 4096);

  analogWriteFrequency(fan1ControlPin,25000);
  analogWriteFrequency(fan2ControlPin,25000);
  analogWriteResolution(10);

  analogReadAveraging(16);
  analogReadRes(10); 

  ambientTemp.begin();
}

void loop() {
  
  if (knobTimer > 100) {
    knobOneValue += knobOne.read() / 4;
    knobTwoValue += knobTwo.read() / 4;
    knobOne.write(0);
    knobTwo.write(0);

    switch (knobMode)
    {
    case 0:
      fan1Setpoint = knobTwoValue;
      fan2Setpoint = knobTwoValue;
      break;
    default:
      break;
    }
    knobTimer -= 100;
  }
  

  /*
  if (fan1freq.available()) {
     Serial.print(fan1freq.read());
  }
  if (fan2freq.available()) {
     Serial.print(fan2freq.read());
  } */
  if (airTempTimer > 5000) {
    ambientTemp.requestTemperatures();
    airTemp = ambientTemp.getTempCByIndex(0);
    //airTemp = 20;
    airTempTimer -= 5000;
  }

  float temp1 = (double)getThermistorTemp(analogRead(temp1InputPin), THERMISTOR_BarrowTherm) / 1000;
  float temp2 = (double)getThermistorTemp(analogRead(temp2InputPin), THERMISTOR_BarrowTherm) / 1000;
  fan1Input = temp1 - (double)airTemp;
  fan2Input = temp2 - (double)airTemp; 
  
  

  fan1PID.Compute();
  fan2PID.Compute();
   
  //analogWrite(fan1ControlPin, fan1Output);
  //analogWrite(fan2ControlPin, fan2Output);
  analogWrite(fan1ControlPin, 350);
  analogWrite(fan2ControlPin, 350);

  if (displayTimer > 1000) {
    Serial.print("AirTemp: "); Serial.println(airTemp);
    Serial.print("Raw1: "); Serial.println(analogRead(temp1InputPin));
    Serial.print("Raw2: "); Serial.println(analogRead(temp2InputPin));
    Serial.print("temp1: "); Serial.print(temp1); Serial.print(" -- "); Serial.println(fan1Input); 
    Serial.print("temp2: "); Serial.print(temp2); Serial.print(" -- "); Serial.println(fan2Input);
    Serial.print("fan1Out: "); Serial.println(fan1Output);
    Serial.print("fan2Out: "); Serial.println(fan2Output);
    displayTimer -= 1000;
    
  }

  
  
}





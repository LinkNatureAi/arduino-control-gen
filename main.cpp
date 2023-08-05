#include <Arduino.h>
#include <Servo.h>

const int supplyStatusPin = 2; // connect adapter + and second pin GND
const int generatorStartPin = 10;
const int generatorRunPin = 11;
const int analogInputPin = A0; // connect battery sensor pin
const float thresholdValue = 21.5;
const unsigned long generatorStartDuration = 1000;
const unsigned long generatorRunDuration = 1000;
const unsigned long servoDelay = 2000;
const unsigned long supplyCheckInterval = 7200000; // 2 hours

bool status = false;
bool supplyAvailable = true;
unsigned long lastSupplyCheckTime = 0;

float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0; // connect battery + 
float R2 = 7500.0; // connect battery - 

Servo servoMotor;

void supplyStatusChange() {
  if (digitalRead(supplyStatusPin) == LOW) {
    supplyAvailable = false;
  } else {
    supplyAvailable = true;
  }
}

void setup() {
  pinMode(supplyStatusPin, INPUT);
  pinMode(generatorStartPin, OUTPUT);
  pinMode(generatorRunPin, OUTPUT);

  digitalWrite(generatorStartPin, LOW);
  digitalWrite(generatorRunPin, LOW);

  attachInterrupt(digitalPinToInterrupt(supplyStatusPin), supplyStatusChange, CHANGE);

  servoMotor.attach(9);
  servoMotor.write(0);
}

void loop() {
  unsigned long currentTime = millis();

   adc_voltage  = (analogRead(analogInputPin)* 5.0) / 1024.0; 
   in_voltage = adc_voltage / (R2/(R1+R2)); 

  if (in_voltage > thresholdValue && !status) {
    servoMotor.write(90);
    delay(servoDelay);
    servoMotor.write(0);
    digitalWrite(generatorStartPin, HIGH);
    delay(generatorStartDuration);
    digitalWrite(generatorStartPin, LOW);
    status = true;
  }

  if ((!supplyAvailable || currentTime - lastSupplyCheckTime >= supplyCheckInterval) && status) {
    digitalWrite(generatorRunPin, HIGH);
    delay(generatorRunDuration);
    digitalWrite(generatorRunPin, LOW);
    status = false;
    supplyAvailable = true;
    lastSupplyCheckTime = currentTime;
  }
}

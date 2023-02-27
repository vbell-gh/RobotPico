#include <Arduino.h>

const int utlraPin = 1; // ultra soninc sensor PIN
const int motorOneInput1 = 26; //input  one of the L293
const int motorOneInput2 = 27; //input  two of the L293
const int motorTwoInput1 = 3; //input  three of the L293
const int motorTwoInput2 = 4; //input  four of the L293
const int pwmMotorOne = 28; // PWM pin for dc motor one
const int pwmMotorTwo = 2; // PWM pin dc motor two

const int voltageRegUlstra = 29; // PIN for the voltage regulator for the ulstrasonic sensor

long duration;
long distanceInCm;
long durationInSeconds;

void setup() {
  Serial.begin(115200);
  pinMode(motorOneInput1, OUTPUT);
  pinMode(motorOneInput2, OUTPUT);
  pinMode(motorTwoInput1, OUTPUT);
  pinMode(motorTwoInput2, OUTPUT);
  pinMode(pwmMotorOne, OUTPUT);
  pinMode(pwmMotorTwo, OUTPUT);


  digitalWrite(motorOneInput1, LOW);
  digitalWrite(motorOneInput2, LOW);
  digitalWrite(motorTwoInput1, LOW);
  digitalWrite(motorTwoInput2, LOW);
 
}

int distance() {
  pinMode(utlraPin, OUTPUT);
  digitalWrite(utlraPin, LOW);
  delayMicroseconds(2);
  digitalWrite(utlraPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(utlraPin, LOW);

  pinMode(utlraPin, INPUT);
  duration = pulseIn(utlraPin, HIGH);
  distanceInCm = (duration * 0.0343) / 2;

  return distanceInCm;
  }

void moveDcFwd(){
  digitalWrite(pwmMotorOne, HIGH);
  digitalWrite(motorOneInput1, LOW);
  digitalWrite(motorOneInput2, HIGH);

  digitalWrite(pwmMotorTwo, HIGH);
  digitalWrite(motorTwoInput1, HIGH);
  digitalWrite(motorTwoInput2, LOW);
}

void moveDcBack(){
  digitalWrite(pwmMotorOne, HIGH);
  digitalWrite(motorOneInput1, HIGH);
  digitalWrite(motorOneInput2, LOW);

  digitalWrite(pwmMotorTwo, HIGH);
  digitalWrite(motorTwoInput1, LOW);
  digitalWrite(motorTwoInput2, HIGH);

}
void stopDC(){
  digitalWrite(pwmMotorOne, LOW);
  digitalWrite(motorOneInput1, LOW);
  digitalWrite(motorOneInput2, LOW);

  digitalWrite(pwmMotorTwo, LOW);
  digitalWrite(motorTwoInput1, LOW);
  digitalWrite(motorTwoInput2, LOW);
}

void loop() {
  distanceInCm = distance();
  Serial.println(distanceInCm);

  if (distanceInCm < 20){
    moveDcBack();
    delay(0.1);
  }

  else if (distanceInCm > 20){
    moveDcFwd();
    delay(0.1);
  }
}
#include <Servo.h>

Servo myServo;

#define servoPin 3
#define actuatorShaftLengthMM 50

// move actuator to the mm position specified
void moveActuator(float strokeMM) {
  float strokePercentage = strokeMM/actuatorShaftLengthMM;

  // don't go too close to servo limit to prevent strain
  if (strokePercentage < 0.01) {
    strokePercentage = 0.01;
  } else if (strokePercentage > 0.99) {
    strokePercentage = 0.99;
  }

  // full range runs from 1000-2000 usec
  int usec = 1000 + strokePercentage * 1000;

  // move actuator
  myServo.writeMicroseconds(usec);
}

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.writeMicroseconds(1000);
}

void loop() {
  int actuatorStep = 2;
  int delayMS = 50;

  // move actuator up to the end
  for (int i = 1; i < 49; i += actuatorStep) {
    moveActuator(i);
    delay(delayMS);
  }

  // move actuator down to the beginning
  for (int i = 50; i > 1; i -= actuatorStep) {
    moveActuator(i);
    delay(delayMS);
  }
}
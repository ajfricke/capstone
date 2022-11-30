#include <Servo.h>

Servo myServo;

#define servoPin 8
#define actuatorShaftLengthMM 100 // change depending on actuator

// move actuator to the mm position specified
void moveActuator(float strokeMM) {
  float strokePercentage = strokeMM/actuatorShaftLengthMM/100;

  // don't go too close to servo limit to prevent strain
  if (strokePercentage < 1.0) {
    strokePercentage = 1.0;
  } else if (strokePercentage > 99.0) {
    strokePercentage = 99.0;
  }

  // full range runs from 1000-2000 usec
  int usec = 1000 + strokePercentage * 1000;

  // move actuator
  myServo.writeMicroseconds(usec);
}

void setup() {
  myServo.attach(servoPin);
}

void loop() {
  int actuatorStep = 5;
  int delayMS = 500;

  // move actuator up to the end
  for (int i = 1; i < 99; i += actuatorStep) {
    moveActuator(i);
    delay(delayMS);
  }

  // move actuator down to the beginning
  for (int i = 99; i > 1; i -= actuatorStep) {
    moveActuator(i);
    delay(delayMS);
  }
}

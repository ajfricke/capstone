#include <Servo.h>

Servo myServo;

#define servoPin 3
#define optocouplerPin A2
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
  // with these settings, will take ~2.5 seconds to move 100 mm
  int strokeStepMM = 2;
  int delayMS = 500;

  // move actuator up to at most 100 mm, checking for laser cut from optocoupler every step 2mm step
  for (int strokeMM = 1; strokeMM < 50; strokeMM += strokeStepMM) {
    int optoVal = analogRead(optocouplerPin); // read the value from the optocoupler
    Serial.println(optoVal); // print the sensor value to the serial monitor

    if (optoVal < 100) {
      Serial.println("Laser cut. Resetting actuator");
      break; // exit loop
    }

    moveActuator(strokeMM);
    delay(delayMS);
  }

  Serial.println("Resetting actuator");
  moveActuator(0); // reset if laser not cut
  delay(2000);
}
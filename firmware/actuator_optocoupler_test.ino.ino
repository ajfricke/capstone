#include <Servo.h>

Servo myServo;

#define servoPin 8
#define optocouplerPin = A2
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
  Serial.begin(9600);
  myServo.attach(servoPin);
}

void loop() {
  // with these settings, will take ~5 seconds to move 100 mm
  int strokeStepMM = 2;
  int delayMS = 100;

  // move actuator up to at most 100 mm, checking for laser cut from optocoupler every step 2mm step
  for (int strokeMM = 1; strokeMM < 100; strokeMM += strokeStepMM) {
    int optoVal = analogRead(optocouplerPin); // read the value from the optocoupler
    Serial.println(optoVal); // print the sensor value to the serial monitor

    if (optoVal < 100) {
      Serial.println("Laser cut. Resetting actuator");
      moveActuator(0); // reset actuator
      
      break; // exit loop
    }

    moveActuator(i);
    delay(delayMS);
  }

  Serial.println("Laser not cut. Resetting actuator");
  moveActuator(0); // reset if laser not cut
  delay(2000);
}

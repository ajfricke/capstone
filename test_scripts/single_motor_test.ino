#include <AccelStepper.h>

// define pins
#define dirPin100 2
#define stepPin100 3
#define dirPin300 4
#define stepPin300 5
#define motorInterfaceType 1 // indicates a driver

#define mmPerStep 5

AccelStepper stepper100 = AccelStepper(motorInterfaceType, stepPin100, dirPin100);
AccelStepper stepper300 = AccelStepper(motorInterfaceType, stepPin300, dirPin300);

int mmToSteps(int mmDistance) {
  return int(mmDistance/mmPerStep)
}

void setup() {
  // setting max speed of the stepper motors
  stepper100.setMaxSpeed(10);
  stepper300.setMaxSpeed(10);

  // setting the acceleration of the stepper motors
  // stepper100.setAcceleration(10);
  // stepper300.setAcceleration(10);
}

void loop() {
  // make 100mm rail travel full distance then back
  stepper100.moveTo(mmToSteps(100));
  stepper100.runToPosition();
  delay(1000);

  // go back to the starting position
  stepper100.moveTo(0);
  stepper100.runToPosition();

  // make 300mm rail travel full distance then back
  stepper300.moveTo(mmToSteps(300));
  stepper300.runToPosition();
  delay(1000);

  stepper300.moveTo(0);
  stepper300.runToPosition();
  
  // make 100mm rail travel half distance
  stepper100.moveTo(mmToSteps(50));
  stepper100.runToPosition();

  // make 300mm rail travel half distance
  stepper300.moveTo(mmToSteps(150));
  stepper300.runToPosition();
  delay(1000);

  // return both rails to starting positions
  stepper100.moveTo(0);
  stepper100.runToPosition();
  stepper300.moveTo(0);
  stepper300.runToPosition();
  delay(1000);

  // move both rails forwards 20mm
  stepper100.moveTo(mmToSteps(20));
  stepper100.runToPosition();
  stepper300.moveTo(mmToSteps(20));
  stepper300.runToPosition();
  delay(1000);

  // move both rails forwards 40mm
  stepper100.moveTo(mmToSteps(40));
  stepper100.runToPosition();
  stepper300.moveTo(mmToSteps(40));
  stepper300.runToPosition();
  delay(1000);

  // move both rails backwards 30mm
  stepper100.moveTo(mmToSteps(-30));
  stepper100.runToPosition();
  stepper300.moveTo(mmToSteps(-30));
  stepper300.runToPosition();
  delay(1000);

  // return both rails to starting positions
  stepper100.moveTo(0);
  stepper100.runToPosition();
  stepper300.moveTo(0);
  stepper300.runToPosition();
  delay(1000);
}

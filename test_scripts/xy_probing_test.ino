#include <AccelStepper.h>

// define pins
#define dirPin100 2
#define stepPin100 3
#define dirPin300 4
#define stepPin300 5
#define motorInterfaceType 1 // indicates a driver

// conversion information
#define mmPerRev 5
#define oneMicrostepStepsPerRev 200
#define microstep 1/4 // change if needed

// hardcoded now, will be determined by app later
#define sex 1 // 0 for men, 1 for women
#define footsize 10 

// example 1st metatarsal (x,y) coordinates for different foot sizes (to be changed)
float m1CoordList[][] = {{5,5}, {10,10}, {15,15}, {20,20}, {25,25}, {30,30}, {35,35}, {40,40}, {45,45}, {50,50}};

// example conversion factors to get the other probing locations (to be changed)
float m3Conversions[] = {2,2};
float m5Conversions[] = {2,2};
float bigToeConversions[] = {2,2};

AccelStepper stepper100 = AccelStepper(motorInterfaceType, stepPin100, dirPin100);
AccelStepper stepper300 = AccelStepper(motorInterfaceType, stepPin300, dirPin300);

// convert distances in mm to motor steps
int mmToSteps(int mmPos) {
  stepsPerRev = oneMicrostepStepsPerRev * 1/microstep
  return int(stepsPerRev*mmPos/mmPerRev);
}

// move the rails to the xy coordinate
void moveXYRails(int xPos, int yPos) {
  stepper300.moveTo(mmToSteps(xPos));
  stepper100.moveTo(mmToSteps(yPos));
  stepper300.runToPosition();
  stepper100.runToPosition();
}

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(5)); // randomize seed using noise from analog pin 5

  // setting max speed and acceleration of the stepper motors to take their max possible values
  stepper100.setMaxSpeed(999999);
  stepper300.setMaxSpeed(999999);
  stepper100.setAcceleration(999999);
  stepper300.setAcceleration(999999);

  // determine the coordinates for each probing location
  float m1Coords[] = m1CoordList[footsize+sex-5];
  float m3Coords[] = {m1Coords[0]*m3Conversions[0], m1Coords[1]*m3Conversions[1]};
  float m5Coords[] = {m1Coords[0]*m5Conversions[0], m1Coords[1]*m5Conversions[1]};
  float bigToeCoords[] = {m1Coords[0]*bigToeConversions[0], m1Coords[1]*bigToeConversions[1]};
  float probingCoords[][] = {m1Coords, m3Coords, m5Coords, bigToeCoords};

  // shuffle the sequence of probing locations
  for (int i = 0; i < 4; i++) {
      int j = random(0, n - i); // get random index

      // swap values
      int t = probingCoords[i];
      probingCoords[i] = probingCoords[j];
      probingCoords[j] = t;
  }
}

void loop() {
  // move rails to each of the 4 probing locations
  for (int i = 0; i < 4; i++) {
    moveXYRails(probingCoords[i][0], probingCoords[i][1])
    delay(3000);
  }

  moveXYRails(0, 0); // return rails to starting position
  exit(0); // exit the loop
}

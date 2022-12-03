#include <AccelStepper.h>
#include <Servo.h>

Servo myServo;

// define pins
#define dirPin100 2
#define stepPin100 3
#define dirPin300 4
#define stepPin300 5
#define motorInterfaceType 1 // indicates a driver
#define servoPin 6 // TODO: check later
#define optocouplerPin A2

#define mmPerRev 5
#define oneMicrostepStepsPerRev 200
#define microstep 1/4 // change if needed
#define actuatorShaftLengthMM 50

// for actuator probing
// these settings will cause it to take 2.5 seconds to raise monofilament
#define strokeStepMM 2
#define delayMS 100

// TODO: hardcoded now, will be determined by app later
#define sex 1 // 0 for men, 1 for women
#define footsize 10 
int foot = 0; // 0 for right, 1 for left

// 1st metatarsal (x,y) coordinates for different foot sizes
// TODO: remove W5 (first) from this list, not needed
float m1CoordList[10][2] = {{17.86979,158.3}, {18.52799,163.83845}, {19.41122,171.12136}, {20.0718,177.67446}, 
                          {21.39691,184.95694}, {21.83603,189.32687}, {22.49935,196.60877}, {22.93777,203.16239}, 
                          {24.03872,208.2592}, {24.70276,214.08506}};

// conversion factors to get the other probing locations
float m3Conversions[2] = {2.666243932,1.004125611};
float m5Conversions[2] = {4.166072072,0.962256948};
float bigToeConversions[2] = {1,1.231986944};

float newReferencePoint[2] = {55, 181.54159};

float w5FootsizeCoords[4][2] = {{17.86979, 158.3}, {47.63918, 157.93545}, {74.43552, 151.35002}, {17.86979, 193.75}};

float m1Coords[2];
float m3Coords[2];
float m5Coords[2];
float bigToeCoords[2];
float probingCoords[4][2];

AccelStepper stepper100 = AccelStepper(motorInterfaceType, stepPin100, dirPin100);
AccelStepper stepper300 = AccelStepper(motorInterfaceType, stepPin300, dirPin300);

// convert distances in mm to motor steps
int mmToSteps(int mmPos) {
  float stepsPerRev = oneMicrostepStepsPerRev * 1/microstep;
  return int(stepsPerRev*mmPos/mmPerRev);
}

// move the rails to the xy coordinate
void moveXYRails(int xPos, int yPos) {
  stepper300.moveTo(mmToSteps(xPos));
  stepper100.moveTo(mmToSteps(yPos));
  stepper300.runToPosition();
  stepper100.runToPosition();
}

// move actuator to the mm position specified
void moveActuator(float strokeMM) {
  float strokePercentage = strokeMM/actuatorShaftLengthMM;

  // don"t go too close to servo limit to prevent strain
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

bool raiseMonofilament() {
  // move actuator up to at most 50 mm, checking for laser cut from optocoupler every strokeStepMM
  for (int mmLocation = 1; mmLocation < actuatorShaftLengthMM; mmLocation += strokeStepMM) {
    int optoVal = analogRead(optocouplerPin); // read the value from the optocoupler

    if (optoVal < 100) {
      return true;
    }

    moveActuator(mmLocation); // move actuator to mmLocation
    delay(delayMS); // delay by delayMS before next stroke
  }

  moveActuator(0); // reset if laser not cut
  return false;
}

// move motors and actuator back to starting positions and exit
void resetAndExit() {
  moveXYRails(0, 0);
  moveActuator(0);
  exit(0);
}

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(5)); // randomize seed using noise from analog pin 5

  // set up actuator
  myServo.attach(servoPin);
  myServo.writeMicroseconds(1000);

  // set up stepper motors
  stepper100.setMaxSpeed(999999);
  stepper300.setMaxSpeed(999999);
  stepper100.setAcceleration(999999);
  stepper300.setAcceleration(999999);

  // determine the coordinates for each probing location
  if ((sex == 1) && (footsize == 5)) {
    for (int i; i < 2; i++) {
      m1Coords[i] = w5FootsizeCoords[0][i];
      m3Coords[i] = w5FootsizeCoords[1][i];
      m5Coords[i] = w5FootsizeCoords[2][i];
      bigToeCoords[i] = w5FootsizeCoords[3][i];
    }
  } else {
    for (int i; i < 2; i++) {
      m1Coords[i] = m1CoordList[footsize+sex-5][i];
      m3Coords[i] = m1Coords[i]*m3Conversions[i];
      m5Coords[i] = m1Coords[i]*m5Conversions[i];
      bigToeCoords[i] = m1Coords[i]*bigToeConversions[i];
    }
  }

  for (int i; i < 2; i++) {
      probingCoords[0][i] = m1Coords[i];
      probingCoords[1][i] = m3Coords[i];
      probingCoords[2][i] = m5Coords[i];
      probingCoords[3][i] = bigToeCoords[i];
  }

  // recalculate probing coordinates with new reference point
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
        probingCoords[i][j] = probingCoords[i][j] - newReferencePoint[j];
    }
  }

  // shuffle the sequence of probing locations
  int j;
  float temp[2];
  for (int i = 0; i < 4; i++) {
      j = random(0, 3 - i); // get random index

      // swap values
      for (int k; k < 2; k++) {
        temp[k] = probingCoords[i][k];
        probingCoords[i][k] = probingCoords[j][k];
        probingCoords[j][k] = temp[k];
      }
  }
}

void loop() {
  Serial.println("Starting test.");
  // move rails to each of the 4 probing locations
  for (int i = 0; i < 4; i++) {
    Serial.println("Moving XY rails to probing location.");

    // negate x-coordinate if left foot
    if (foot == 1) {
      moveXYRails(-probingCoords[i][0], probingCoords[i][1]);
    } else {
      moveXYRails(probingCoords[i][0], probingCoords[i][1]);
    }

    // test probing location three times
    int improperProbeCount = 0;
    for (int j = 0; j < 3; j++) {
        Serial.println("Raising monofilament.");
        bool properProbe = raiseMonofilament();

        if (properProbe == false) {
            j -= 1;
            improperProbeCount += 1;
        } else {
            delay(2000); // hold probe to foot for 2 seconds
        }

        if (improperProbeCount == 3){
            Serial.println("Monofilament is not exerting 10g force as determined by optocoupler after 3 tries. Exiting.");
            resetAndExit();
        }

        Serial.println("Raising lowering.");
        moveActuator(0); // lower monofilament back down

        if (j != 2) {
            delay(random(1000, 3000)); // randomizes same location probing at intervals between 1-3s
        }
    }

    delay(random(1000, 3000)); // randomizes different location probing at intervals between 1-3s
  }

  if (foot == 1) {
    Serial.println("Test finished. Exiting.");
    resetAndExit();
  } else {
    foot = 1; // test left foot next
    moveXYRails(0, 0);
    moveActuator(0);
    Serial.println("Repeating test with left foot.");
  }
}
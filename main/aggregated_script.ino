#include <AccelStepper.h>
#include <Servo.h>

// TODO: ADD BUTTON DEBOUNCING

Servo myServo;

//** PINS & VARIABLES FOR ALL **//
int mode = 0; // 0 for waiting, 1 for calibration, 2 for probing
//** END **//

//** PINS & VARIABLES FOR PROBING **//
// define pins
#define dirPin100 4
#define stepPin100 5
#define dirPin300 7
#define stepPin300 6
#define motorInterfaceType 1 // indicates a driver
#define servoPin 10 // TODO: check later
#define optocouplerPin A2

#define mmPerRev 5
#define oneMicrostepStepsPerRev 200
#define microstep 0.25 // change if needed
#define actuatorShaftLengthMM 50

// for actuator probing
#define strokeStepMM 2
#define delayMS 100

// TODO: hardcoded now, will be determined by app later
#define sex 1 // 0 for men, 1 for women
#define footsize 10 
//** END ** //

//** PINS & VARIABLES FOR CALIBRATION **//
#define buttonRail 2
#define buttonIn 3
#define buttonOut 4 // TODO: DUPLICATE PIN - change
#define clockPin 9
#define latchPin 10 // TODO: DUPLICATE PIN - change
#define dataPin 12

bool moveLED;

int buttonStateOut;
int buttonStateIn;
int buttonStateRail;

int sr_i;
int sr_j;
int maxPos;

String currState;
String prevState;
int prevPos;

int leftLEDs[10][4] = {{0,0,0,1}, {0,0,0,2}, {0,0,0,4}, {0,0,0,8},
					   {0,0,0,16}, {0,0,0,32}, {0,0,0,64}, {0,0,0,128},
                  	   {0,0,1,0}, {0,0,2,0}};

int rightLEDs[10][4] = {{0,0,4,0}, {0,0,8,0}, {0,0,16,0}, {0,0,32,0}, 
					    {0,0,64,0}, {0,0,128,0}, {0,1,0,0}, {0,2,0,0}, 
					    {0,4,0,0}, {0,8,0,0}};

int leftMidLEDs[4][4] = {{1,0,0,0}, {2,0,0,0}, {4,0,0,0}, {8,0,0,0}};

int rightMidLEDs[4][4] = {{16,0,0,0}, {32,0,0,0}, {64,0,0,0}, {128,0,0,0}};

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
//** END ** //


// convert distances in mm to motor steps
int mmToSteps(int mmPos) {
  float stepsPerRev = oneMicrostepStepsPerRev * 1/microstep;
  int steps = int(stepsPerRev*mmPos/mmPerRev);
  return steps;
}

// move the rails to the xy coordinate
void moveXYRails(int xPos, int yPos) {
  stepper100.moveTo(mmToSteps(xPos));
  stepper300.moveTo(mmToSteps(yPos));
  stepper100.runToPosition();
  stepper300.runToPosition();
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

bool calibrationLoop() {
    // read buttons; TODO: change to wait for app input
    moveLED = false;

    buttonStateOut = digitalRead(buttonOut);
    buttonStateIn = digitalRead(buttonIn);
    buttonStateRail = digitalRead(buttonRail);
    
    if (buttonStateOut == HIGH) { // move LED outwards
        if (sr_i != maxPos) {
            sr_i++;
        } else {
            sr_i = 0;
        }
        moveLED = true;
        delay(150);
    } else if (buttonStateIn == HIGH) { // move LED inwards
        if (sr_i != 0) {
            sr_i--;
        } else {
            sr_i = maxPos;
        }
        moveLED = true;
        delay(150);
    } else if (buttonStateRail == HIGH) { // change LED rail
        // TODO: send i position of currState to app
        if (currState == "left") {
            maxPos = 3;
            currState = "mid";
            prevState = "left";
            prevPos = sr_i;
        } else if (currState == "right") {
            maxPos = 3;
            currState = "mid";
            prevState = "right";
            prevPos = sr_i;
        // something to think about: this won't be "change rail" when mid, would be something like "confirm"
        // actually, I guess it would be "confirm" for all of them
        // have option to go back to left/right after confirming mid
        } else if (currState == "mid") {
            maxPos = 9;
            if (prevState == "right") {
                // currState = "left";
                // TODO: send signal to app to notify that we're done, or app assumes based on number of positions received
                return true;
            } else if (prevState == "left") {
                currState = "right";
            }
        }
        sr_i = 0;
        delay(150);
    }

    // move the LED
    if (moveLED) {
        digitalWrite(latchPin, LOW);
        if (currState == "left") {
            shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[sr_i][0]);
            shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[sr_i][1]);
            shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[sr_i][2]);
            shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[sr_i][3]);
        } else if (currState == "right") {
            shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[sr_i][0]);
            shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[sr_i][1]);
            shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[sr_i][2]);
            shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[sr_i][3]);
        } else if (currState == "mid") {
            if (prevState == "left") {
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[sr_i][0] + leftLEDs[prevPos][0]);
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[sr_i][1] + leftLEDs[prevPos][1]);
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[sr_i][2] + leftLEDs[prevPos][2]);
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[sr_i][3] + leftLEDs[prevPos][3]);
            } else if (prevState == "right") {
            sr_j = 3-sr_i;
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[sr_j][0] + rightLEDs[prevPos][0]);
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[sr_j][1] + rightLEDs[prevPos][1]);
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[sr_j][2] + rightLEDs[prevPos][2]);
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[sr_j][3] + rightLEDs[prevPos][3]);
            }
        }
        digitalWrite(latchPin, HIGH);
    }
    return false;
}

void probeLoop() {
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
            Serial.println("Input from photointerruptor. Holding for 2 seconds");
            delay(2000); // hold probe to foot for 2 seconds
        }

        if (improperProbeCount == 3){
            Serial.println("Monofilament is not exerting 10g force as determined by optocoupler after 3 tries. Exiting.");
            resetAndExit();
        }

        Serial.println("Lowering monofilament.");
        moveActuator(0); // lower monofilament back down

        if (j != 2) {
            delay(random(500, 2000)); // randomizes timing between same location probing at intervals between 1-3s
        }
    }

    delay(random(500, 2000)); // randomizes timing between different location probing at intervals between 1-3s
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

void setup() {
  //** PROBING SETUP **//
  Serial.begin(9600);
  randomSeed(analogRead(5)); // randomize seed using noise from analog pin 5

  // set up actuator
  myServo.attach(servoPin);
  myServo.writeMicroseconds(1000);

  // set up stepper motors
  stepper100.setMaxSpeed(50000);
  stepper300.setMaxSpeed(50000);
  stepper100.setAcceleration(50000);
  stepper300.setAcceleration(50000);

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
  //** END **//

  //** CALIBRATION SETUP **//
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  
  pinMode(buttonOut, INPUT);
  pinMode(buttonIn, INPUT);
  pinMode(buttonRail, INPUT);
  //** END **//
}

void loop() {
    // check for input from app
    // change mode based on input
    bool bothFeet = true; // based on input
    int foot = 0; // 0 for right, 1 for left - based on input; if both, start right otherwise which foot is passed

    if (startCalibration) { // based on input
        sr_i = 0;
        maxPos = 9;
        currState = "left"; // actually, change this based on foot
    }

    // control currState here instead and pass in to calibrationLoop()
    if (mode == 1) {
        // REFACTOR LOOP TO TAKE LEFT OR RIGHT
        if (calibrationLoop()) {
            // if calibration finishes, start waiting
            mode = 0;
        }
    } else if (mode == 2) {
        // how about we pass in left/right
        if (probeLoop()) {
            // once probing finishes
            if (bothFeet AND foot == 0) {
                // proceed to the next foot
                foot = 1;
            } else {
                // start waiting
                mode = 0
            }
        }
        probeLoop();
    }
}

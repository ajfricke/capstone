#include <AccelStepper.h>
#include <Servo.h>

//** GENERAL INITIALIZATIONS **//
#define rxPin 2
#define txPin 3
SoftwareSerial serialComm = Bluetooth(txPin, rxPin);

Servo myServo;
int mode = 0; // 0 for waiting, 1 for calibration, 2 for probing

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

String footInfo;
String toPerform;

bool newData = false;
bool recvdAppData = true;
//** END **//


//** CALIBRATION INITIALIZATIONS **//
#define clockPin 9
#define latchPin 11
#define dataPin 12

int sex; // 0 for female, 1 for male
int footsize;

// TODO: temporary. Update with actual values
int footsizeToLED[10][2] = {{0, 0}, {1, 0}, {2, 0}, {3, 1}, {4, 1}, {5, 1}, {6, 2}, {7, 2}, {8, 3}, {9, 3}}

int leftLEDs[10][4] = {{0,0,0,1}, {0,0,0,2}, {0,0,0,4}, {0,0,0,8},
					   {0,0,0,16}, {0,0,0,32}, {0,0,0,64}, {0,0,0,128},
                  	   {0,0,1,0}, {0,0,2,0}};

int rightLEDs[10][4] = {{0,0,4,0}, {0,0,8,0}, {0,0,16,0}, {0,0,32,0}, 
					    {0,0,64,0}, {0,0,128,0}, {0,1,0,0}, {0,2,0,0}, 
					    {0,4,0,0}, {0,8,0,0}};

int leftMidLEDs[4][4] = {{1,0,0,0}, {2,0,0,0}, {4,0,0,0}, {8,0,0,0}};

int rightMidLEDs[4][4] = {{16,0,0,0}, {32,0,0,0}, {64,0,0,0}, {128,0,0,0}};

int leftLEDPos = -1;
int rightLEDPos = -1;
int leftMidLEDPos = -1;
int rightMidLEDPos = -1;
//** END ** //


//** PROBING INITIALIZATIONS **//
// define pins
#define dirPin100 4
#define stepPin100 5
#define dirPin300 7
#define stepPin300 6
#define motorInterfaceType 1 // indicates a driver
#define servoPin 10
#define interrupterPin1 A2
#define interrupterPin2 A3

#define mmPerRev 5
#define oneMicrostepStepsPerRev 200
#define microstep 0.25 // change if needed
#define actuatorShaftLengthMM 50

// for actuator probing
#define strokeStepMM 2
#define delayMS 100

bit response;
unsigned long currMillis;
unsigned long prevMillis;

// 1st metatarsal (x,y) coordinates for different foot sizes
float m1CoordList[9][2] = {{18.52799,163.83845}, {19.41122,171.12136}, {20.0718,177.67446}, 
                                {21.39691,184.95694}, {21.83603,189.32687}, {22.49935,196.60877}, 
                                {22.93777,203.16239}, {24.03872,208.2592}, {24.70276,214.08506}};

// conversion factors to get the other probing locations
float m3Conversions[2] = {2.666243932,1.004125611};
float m5Conversions[2] = {4.166072072,0.962256948};
float bigToeConversions[2] = {1,1.231986944};
float newReferencePoint[2] = {55, 181.54159};
float w5FootsizeCoords[4][2] = {{17.86979, 158.3}, {47.63918, 157.93545}, {74.43552, 151.35002}, {17.86979, 193.75}};
float probingCoords[4][2];

AccelStepper stepper100 = AccelStepper(motorInterfaceType, stepPin100, dirPin100);
AccelStepper stepper300 = AccelStepper(motorInterfaceType, stepPin300, dirPin300);
//** END **//


//** CALIBRATION FUNCTIONS **//
void initializeLEDs(String foot, int verticalPos, int horizPos) {
    digitalWrite(latchPin, LOW);
    if (foot == "left") {
        // light up LEDs for left foot
        for (int i = 0; i < 4; i++) {
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[horizPos][i] + leftLEDs[verticalPos][i]);
        }
    } else if (foot == "right") {
        // light up LEDs for right foot
        for (int i = 0; i < 4; i++) {
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[horizPos][i] + rightLEDs[verticalPos][i]);
        }
    }
    digitalWrite(latchPin, HIGH);
}


void calibrationLoop(String currState, int verticalStartPos, int horizStartPos) {
    int sr_i = verticalStartPos;
    int sr_j;
    int maxPos = 9
    int prevPos;
    bool finished = false;
    bool moveLED = false;
    String prevState;
    byte instruction = -1;

    while (finished == false) {
        Serial.println('Starting calibration.')
        instruction = getAppInput(); 

        if (instruction == -1) {
            continue;
        } else if (instruction == 0) { // move LED inwards
            Serial.println('Moving LED inwards.')
            if (sr_i != 0) {
                sr_i--;
            } else {
                sr_i = maxPos;
            }
            moveLED = true;
        } else if (instruction == 1) { // move LED outwards
            Serial.println('Moving LED outwards.')
            if (sr_i != maxPos) {
                sr_i++;
            } else {
                sr_i = 0;
            }
            moveLED = true;
        } else if (instruction == 2) { // rail confirmed
            if (currState == "left" or currState == "right") {
                Serial.println('Moving to mid LED rail.')
                sr_i = horizStartPos;
                maxPos = 3;
                prevState = currState;
                currState = "mid";
                prevPos = sr_i;
                if (currState == "left") {
                    leftLEDPos = sr_i;
                } else if (currState == "right") {
                    rightLEDPos = sr_i;
                }
            } else if (currState == "mid") {
                Serial.println('Finished calibration.')
                if (prevState == "left") {
                    leftMidLEDPos = sr_i;
                } else if (prevState == "right") {
                    rightMidLEDPos = sr_j;
                }
                finished = true;
            }
        }

        // move the LED
        if (moveLED) {
            digitalWrite(latchPin, LOW);
            if (currState == "left") {
                for (int i = 0; i < 4; i++) {
                    shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[sr_i][i]);
                }
            } else if (currState == "right") {
                for (int i = 0; i < 4; i++) {
                    shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[sr_i][i]);
                }
            } else if (currState == "mid") {
                if (prevState == "left") {
                    for (int i = 0; i < 4; i++) {
                        shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[sr_i][i] + leftLEDs[prevPos][i]);
                    }
                } else if (prevState == "right") {
                    sr_j = 3-sr_i;
                    for (int i = 0; i < 4; i++) {
                        shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[sr_j][i] + rightLEDs[prevPos][i]);
                    }
                }
            }
            digitalWrite(latchPin, HIGH);
        }
    }  
}
//** END **//


//** PROBING FUNCTIONS **//
void getProbingCoords() {
    // determine the coordinates for each probing location
    if ((sex == 0) && (footsize == 5)) {
        for (int k; k < 4; k++) {
            for (int i; i < 2; i++) {
                probingCoords[k][i] = w5FootsizeCoords[k][i];
            }
        }
    } else {
        for (int i; i < 2; i++) {
            probingCoords[0][i] = m1CoordList[footsize+sex-6][i];
            probingCoords[1][i] = probingCoords[0][i]*m3Conversions[i];
            probingCoords[2][i] = probingCoords[0][i]*m5Conversions[i];
            probingCoords[3][i] = probingCoords[0][i]*bigToeConversions[i];
        }
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


// convert distances in mm to motor steps
int mmToSteps(int mmPos) {
    float stepsPerRev = oneMicrostepStepsPerRev * 1/microstep;
    return int(stepsPerRev*mmPos/mmPerRev);
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


// raise actuator until photointerrupter input
bool raiseMonofilament() {
    // move actuator up to at most actuatorShaftLengthMM, checking for input from photointerrupters every strokeStepMM
    for (int mmLocation = 1; mmLocation < actuatorShaftLengthMM; mmLocation += strokeStepMM) {
        int photoVal1 = analogRead(interrupterPin1); // read the value from the first photointerrupter
        int photoVal2 = analogRead(interrupterPin2); // read the value from the first photointerrupter

        if (photoVal1 < 100 || photoVal2 < 100) {
        return true;
        }

        moveActuator(mmLocation); // move actuator to mmLocation
        delay(delayMS); // delay by delayMS before next stroke
    }

    moveActuator(0); // reset if laser not cut
    return false;
}


// move motors and actuator back to starting positions
void resetMotors() {
    moveXYRails(0, 0);
    moveActuator(0);
}


void probeLoop(String foot) {
    Serial.println("Starting probing test.");
    // move rails to each of the 4 probing locations
    for (int i = 0; i < 4; i++) {
        Serial.println("Moving XY rails to probing location.");

        // negate x-coordinate if left foot
        if (foot == "left") {
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
                Serial.println("Input received from the photointerrupters. Holding for 2 seconds");
                delay(2000); // hold probe to foot for 2 seconds
            }

            if (improperProbeCount == 3){
                Serial.println("Monofilament is not exerting 10g force as determined by the photointerrupters after 3 tries. Exiting.");
                serialComm.write(-1); // tell app we failed
                resetMotors();
                return;
            }

            Serial.println("Lowering monofilament.");
            moveActuator(0); // lower monofilament back down


            prevMillis = millis();
            currMillis = millis();
            
            
            // wait for user response, otherwise will be marked as no
            while((currMillis - prevMillis < 2000) || response == 1) {
                response = getAppInput();
                currMillis = millis();
            }

            if (j != 2) {
                delay(random(500, 2000)); // randomize timing between same location probing at intervals between 1-3s
            }
        }
        
        serialComm.write(0); // tell app that we're moving on to next location
        delay(random(500, 2000)); // randomize timing between different location probing at intervals between 1-3s
    }

    Serial.println("Test finished. Resetting motors and exiting.");
    serialComm.write(1); // tell app that the test finished
    resetMotors();
}
//** END **//


//** APP INTERACTION FUNCTIONS **//
bool getInitialAppInput() {
    bool recvInProgress = false;
    byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    // receive data from the app
    while (serialComm.available() > 0 && newData == false) {
        rc = serialComm.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            } else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }

    if (newData == true) {
        strcpy(tempChars, receivedChars);
        char * strtokIndx; // used by strtok() as an index

        toPerform = strtok(tempChars, ","); // probe or calibration
        Serial.println('Got info on what to perform from app: ')
        Serial.print(toPerform);

        footInfo = strtok(NULL, ","); // left, right, or both
        Serial.println('Got foot info from app: ')
        Serial.print(footInfo);

        if (toPerform == 'probe') {
            if (footInfo == "right" || footInfo == "both") {
                rightLEDPos = int(strtok(NULL, ","));
                rightMidLEDPos = int(strtok(NULL, ","));
            }
            if (footInfo == "left" || footInfo == "both") {
                leftLEDPos = int(strtok(NULL, ","));
                leftMidLEDPos = int(strtok(NULL, ","));
            }
        } else if (toPerform == 'calibration') {
            footsize = int(strtok(NULL, ","));
            Serial.println('Got foot size from app: ');
            Serial.print(footsize);

            sex = int(strtok(NULL, ","));
            Serial.println('Got sex from app: ');
            Serial.print(sex);
        }

        newData = false;

        return true;
    }

    return false;
}


byte getAppInput() {
    if (serialComm.available()>0) {
        byte inputByte = serialComm.read();
        Serial.println('Received data from the app: ');
        Serial.print(inputByte);

        return inputByte;
    }
    return -1
}
//** END **//


void setup() {
    // setting up bluetooth module
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    serialComm.begin(9600);
    
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

    // set up shift registers
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
}


void loop() {
    recvdAppInput = getInitialAppInput();

    if (recvdAppInput) {
        if (toPerform == "calibrate") {
            mode = 1;
        } else if (toPerform == "probe") {
            getProbingCoords();
            mode = 2;
        }
    }

    if (mode == 0) { // waiting for app instructions
        continue;
    } else if (mode == 1) { // calibration
        verticalLEDStartPos = footsizeToLED[footsize+sex-5][0];
        horizLEDStartPos = footsizeToLED[footsize+sex-5][1];

        if (footInfo == "both" || footInfo == "right") { // calibrate right foot
            initializeLEDs("right", verticalLEDStartPos, horizLEDStartPos)
            calibrationLoop("right", verticalLEDStartPos, horizLEDStartPos);
        }
        if (footInfo == "both" || footInfo == "left") { // calibrate left foot
            initializeLEDs("left", verticalLEDStartPos, horizLEDStartPos)
            calibrationLoop("left", verticalLEDStartPos, horizLEDStartPos);
        }

        mode = 0; // go back to waiting
    } else if (mode == 2) {
        if (footInfo == "both" || footInfo == "right") { // probe right foot
            initializeLEDs("right", rightLEDPos, rightMidLEDPos);
            while (getAppInput() != 1) { // wait from confirmation from app
                continue;
            }
            probeLoop("right");
        }
        if (footInfo == "both" || footInfo == "left") {
            initializeLEDs("left", leftLEDPos, leftMidLEDPos);
            while (getAppInput() != 1) { // wait from confirmation from app
                continue;
            }
            probeLoop("left");
        }

        mode = 0; // go back to waiting
    }
}

// TODO: implement feature that receives "kill switch" from app
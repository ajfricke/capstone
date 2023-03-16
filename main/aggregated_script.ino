#include <AccelStepper.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define rxBluetoothPin 7 // Green wire
#define txBluetoothPin 6 // Blue wire
SoftwareSerial serialComm = SoftwareSerial(rxBluetoothPin, txBluetoothPin);

Servo myServo;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool recvInProgress = false;
byte ndx = 0;
char rc;
char startMarker = '<';
char endMarker = '>';

String footInfo;
String toPerform;

bool newData = false;
bool recvdAppInput = false;
//** END **//


//** CALIBRATION INITIALIZATIONS **//
#define clockPin 11
#define latchPin 10
#define dataPin 9

byte sex; // 0 for female, 1 for male
byte footsize;

byte footsizeToLED[10][2] = {{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}, {6, 2}, {6, 3}, {7, 3}, {8, 3}};

byte leftLEDs[9][4] = {{0,0,0,128}, {0,0,0,64}, {0,0,0,32}, {0,0,0,16},
                        {0,0,0,8}, {0,0,0,4}, {0,0,0,2}, {0,0,0,1},
                        {0,0,1,0}};

int rightLEDs[9][4] = {{0,1,0,0}, {0,2,0,0}, {0,4,0,0}, {0,8,0,0}, 
                        {0,16,0,0}, {0,32,0,0}, {0,64,0,0}, {0,128,0,0}, 
                        {0,0,128,0}};

int leftMidLEDs[4][4] = {{8,0,0,0}, {4,0,0,0}, {2,0,0,0}, {1,0,0,0}};

int rightMidLEDs[4][4] = {{16,0,0,0}, {32,0,0,0}, {64,0,0,0}, {128,0,0,0}};

byte leftLEDPos = -1;
byte rightLEDPos = -1;
byte leftMidLEDPos = -1;
byte rightMidLEDPos = -1;
//** END ** //


//** PROBING INITIALIZATIONS **//
// define pins
#define dirPin100 2
#define stepPin100 3
#define dirPin300 4
#define stepPin300 5
#define motorInterfaceType 1 // indicates a driver
#define servoPin 13
#define interrupterPin1 A0
#define interrupterPin2 A1

#define mmPerRev 5
#define oneMicrostepStepsPerRev 200
#define microstep 0.25 // change if needed
#define actuatorMaxTravelDist 40
#define actuatorShaftLengthMM 50

// for actuator probing
#define strokeStepMM 2
#define delayMS 100

byte response = -1;
unsigned long currMillis;
unsigned long prevMillis;

// 1st metatarsal (x,y) coordinates for different foot sizes
float m1CoordList[9][2] = {{18.52799,163.83845}, {19.41122,171.12136}, {20.0718,177.67446},  // M5/W6, M6/W7, M7/W8
                                {21.39691,184.95694}, {21.83603,189.32687}, {22.49935,196.60877}, // M8, M9, M10
                                {22.93777,203.16239}, {24.03872,208.2592}, {24.70276,214.08506}}; // M11, M12, M13

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
void resetLEDs() {
    //Serial.println("Resetting LEDs");
    digitalWrite(latchPin, LOW);
    for (byte i = 0; i < 4; i++) {
        shiftOut(dataPin, clockPin, MSBFIRST, 0);
    }
    digitalWrite(latchPin, HIGH);
}

void initializeLEDs(String foot, byte verticalPos, byte horizPos) {
    //Serial.println("Initializing LEDs");
    digitalWrite(latchPin, LOW);
    if (foot == "left") {
        // light up LEDs for left foot
        for (byte i = 0; i < 4; i++) {
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[horizPos][i] + rightLEDs[verticalPos][i]);
        }
    } else if (foot == "right") {
        // light up LEDs for right foot
        for (byte i = 0; i < 4; i++) {
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[horizPos][i] + leftLEDs[verticalPos][i]);
        }
    }
    digitalWrite(latchPin, HIGH);
}


void updateFootsize(byte trueVertPos, byte trueHorizPos, byte estVertPos, byte estHorizPos) {
    byte horizIdxRange[4][2] = {{0, 1}, {2, 3}, {4, 6}, {7, 9}};
    Serial.println("Updating footsize");

    // char printOldFootsize[20];
    // sprintf(printOldFootsize, "Old footsize: %d", footsize);
    // Serial.println(printOldFootsize);

    // char printEstLEDPos[30];
    // sprintf(printEstLEDPos, "Est. LED positions: %d, %d", estVertPos, estHorizPos);
    // Serial.println(printEstLEDPos);

    // char printTrueLEDPos[30];
    // sprintf(printTrueLEDPos, "True LED positions: %d, %d", trueVertPos, trueHorizPos);
    // Serial.println(printTrueLEDPos);
    
    if (trueVertPos == estVertPos && trueHorizPos == estHorizPos) {
        // CASE #1: true positions are equivalent to estimated - no need to update
        footsize = footsize;
    // CASE #2: vertical and horizontal positions line up correctly for a new foot size - update
    } else if ((trueHorizPos < 3) && (footsizeToLED[trueVertPos][1] == trueHorizPos)) { // covers first 7 LED combos
        footsize = trueVertPos-sex+5;
    } else if ((footsizeToLED[trueVertPos+1][1] == 3)) { // covers last 3 LED combos
        footsize = trueVertPos+1-sex+5;

    // CASE #3: vertical and horizontal positions do not line up - update, favouring horizontal position
    } else {
        if ((trueVertPos - footsizeToLED[horizIdxRange[trueHorizPos][0]][0]) == -1) {
            footsize = horizIdxRange[trueHorizPos][0]-sex+5;
        } else if ((trueVertPos - footsizeToLED[horizIdxRange[trueHorizPos][0]][0]) < -1) {
            footsize = horizIdxRange[trueHorizPos-1][0]-sex+5;
        } else if ((trueVertPos - footsizeToLED[horizIdxRange[trueHorizPos][1]][0]) == 1) {
            footsize = horizIdxRange[trueHorizPos][1]-sex+5;
        } else if ((trueVertPos - footsizeToLED[horizIdxRange[trueHorizPos][1]][0]) > 2) {
            footsize = horizIdxRange[trueHorizPos+1][1]-sex+5;
        }
    }
    
    char printNewFootsize[20];
    sprintf(printNewFootsize, "New footsize: %d", footsize);
    Serial.println(printNewFootsize);

    // char printNewLEDPos[30];
    // sprintf(printNewLEDPos, "New LED positions: %d, %d", footsizeToLED[footsize+sex-5][0], footsizeToLED[footsize+sex-5][1]);
    // Serial.println(printNewLEDPos);
}


void calibrationLoop(String foot, byte verticalStartPos, byte horizStartPos) {
    byte verticalPos = verticalStartPos;
    byte horizPos = horizStartPos;
    bool finished = false;
    bool moveLED = false;
    byte instruction = -1;

    while (finished == false) {
        instruction = getAppInput(); 

        if (instruction == -1) {
            continue;
        } else if (instruction == 0) { // move vertical LED down
            //Serial.println("Moving vertical LED downwards.");
            if (verticalPos != 0) {
                verticalPos--;
            } else {
                //Serial.println("Hit first LED. Move upwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 1) { // move vertical LED up
            //Serial.println("Moving vertical LED upwards.");
            if (verticalPos != 8){
                verticalPos++;
            } else {
                //Serial.println("Hit last LED. Move downwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 2) { // move horizontal LED inwards
            //Serial.println("Moving horizontal LED inwards.");
            if (horizPos != 0) {
                horizPos--;
            } else {
                //Serial.println("Hit first LED. Move outwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 3) { // move horizontal LED outwards
            //Serial.println("Moving horizontal LED outwards.");
            if (horizPos != 3) {
                horizPos++;
            } else {
                //Serial.println("Hit first LED. Move outwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 4) { // LEDs confirmed
            //Serial.println("Finished calibration.");
            if (foot == "left") {
                rightLEDPos = verticalPos;
                leftMidLEDPos = horizPos;
            } else if (foot == "right") {
                leftLEDPos = verticalPos;
                rightMidLEDPos = horizPos;
            }
            updateFootsize(verticalPos, horizPos, verticalStartPos, horizStartPos);

            // send data to app
            char dataToArduino[10];
            sprintf(dataToArduino, "%d,%d,%d", footsize, verticalPos, horizPos);
            //Serial.println(dataToArduino);
            serialComm.println(dataToArduino);
            
            finished = true;
        }

        // move the LED
        if (moveLED) {
            digitalWrite(latchPin, LOW);
            if (foot == "left") {
                for (byte i = 0; i < 4; i++) {
                    shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[horizPos][i] + rightLEDs[verticalPos][i]);
                }
            } else if (foot == "right") {
                for (byte i = 0; i < 4; i++) {
                    shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[horizPos][i] + leftLEDs[verticalPos][i]);
                }
            }
            digitalWrite(latchPin, HIGH);
            moveLED = false;
        }
    }  
}
//** END **//


//** PROBING FUNCTIONS **//
void getProbingCoords() {
    //Serial.println("Getting probing coordinates");
    // determine the coordinates for each probing location
    if ((sex == 0) && (footsize == 5)) {
        for (byte k = 0; k < 4; k++) {
            for (byte i = 0; i < 2; i++) {
                probingCoords[k][i] = w5FootsizeCoords[k][i];
            }
        }
    } else {
        for (byte i = 0; i < 2; i++) {
            probingCoords[0][i] = m1CoordList[footsize+sex-6][i];
            probingCoords[1][i] = probingCoords[0][i]*m3Conversions[i];
            probingCoords[2][i] = probingCoords[0][i]*m5Conversions[i];
            probingCoords[3][i] = probingCoords[0][i]*bigToeConversions[i];
        }
    }

    // recalculate probing coordinates with new reference point
    for (byte i = 0; i < 4; i++) {
        for (byte j = 0; j < 2; j++) {
            probingCoords[i][j] = probingCoords[i][j] - newReferencePoint[j];
        }
    }

    // TODO: make sure shuffling is working
    // shuffle the sequence of probing locations
    byte j;
    float temp[2];
    for (byte i = 0; i < 4; i++) {
        j = random(0, 3 - i); // get random index

        for (byte k = 0; k < 2; k++) {
            temp[k] = probingCoords[i][k];
            probingCoords[i][k] = probingCoords[j][k];
            probingCoords[j][k] = temp[k];
        }
    }

    Serial.println("Probing Coordinates:");
    for (byte i = 0; i < 4; i++) {
      Serial.print(probingCoords[i][0], 5);
      Serial.print(" ");
      Serial.println(probingCoords[i][1], 5);
    }
}


// convert distances in mm to motor steps
int mmToSteps(int mmPos) {
    float stepsPerRev = oneMicrostepStepsPerRev * 1/microstep;
    return int(stepsPerRev*mmPos/mmPerRev);
}


// move the rails to the xy coordinate
void moveXYRails(int xPos, int yPos) {
    Serial.print("Moving to: ");
    Serial.print(xPos);
    Serial.print(",");
    Serial.println(yPos);
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
    //Serial.println(strokePercentage);
    myServo.writeMicroseconds(usec);
}


// raise actuator until photointerrupter input
bool raiseMonofilament() {
    // move actuator up to at most actuatorShaftLengthMM, checking for input from photointerrupters every strokeStepMM
    for (int mmLocation = 1; mmLocation < actuatorMaxTravelDist; mmLocation += strokeStepMM) {
        int photoVal1 = analogRead(interrupterPin1); // read the value from the first photointerrupter
        int photoVal2 = analogRead(interrupterPin2); // read the value from the second photointerrupter

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
    Serial.println("Resetting to starting position.");
    moveActuator(0);
    delay(500);
    moveXYRails(0, 0);
}


void probeLoop(String foot) {
    //Serial.println("Starting probing.");
    // move rails to each of the 4 probing locations
    for (byte i = 0; i < 4; i++) {
        //Serial.println("Moving XY rails to probing location.");
        // negate x-coordinate if left foot, negate y-coordinates
        if (foot == "left") {
            moveXYRails(-probingCoords[i][0], -probingCoords[i][1]);
        } else {
            moveXYRails(probingCoords[i][0], -probingCoords[i][1]);
        }

        // test probing location three times
        byte improperProbeCount = 0;
        for (byte j = 0; j < 3; j++) {
            Serial.println("Raising monofilament.");
            bool properProbe = raiseMonofilament();

            if (properProbe == false) {
                j -= 1;
                improperProbeCount += 1;
            } else {
                Serial.println("Detected bend. Holding for 2 sec");

                currMillis = millis();
                prevMillis = millis();
                
                // check for user response
                response = -1;
                while((currMillis - prevMillis < 2000) && response != 1) {
                    response = getAppInput();
                    currMillis = millis();
                }
            }

           if (improperProbeCount == 3){
                Serial.println("Monofilament not picked up by photointerrupters after 3 tries. Exiting.");
                serialComm.write(-1); // tell app we failed
                resetMotors();
                return;
            }

            Serial.println("Lowering monofilament.");
            moveActuator(0); // lower monofilament back down

            currMillis = millis();
            prevMillis = millis();
            
            // wait for user response, otherwise will be marked as no
            response = -1;
            while((currMillis - prevMillis < 2000) && response != 1) {
                response = getAppInput();
                currMillis = millis();
            }

            if (j != 2) {
                delay(random(500, 2000)); // randomize timing between same location probing at intervals between 1-3s
            }
        }
        
        //Serial.println("Moving to next probing location.");
        serialComm.write((byte) 0x00); // tell app that we"re moving on to next location
        delay(random(500, 2000)); // randomize timing between different location probing at intervals between 1-3s
    }

    Serial.println("Test finished.");
    serialComm.write(1); // tell app that the test finished
    resetMotors();
}
//** END **//


//** APP INTERACTION FUNCTIONS **//
bool getAppCommand() {
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

        toPerform = strtok(tempChars, ","); // probe or calibrate
        Serial.print("Mode from app: ");
        Serial.println(toPerform);

        footInfo = strtok(NULL, ","); // left or right
        Serial.print("Foot from app: ");
        Serial.println(footInfo);

        sex = atoi(strtok(NULL, ","));
        Serial.print("Sex from app: ");
        Serial.println(sex);

        footsize = atoi(strtok(NULL, ","));
        Serial.print("Foot size from app: ");
        Serial.println(footsize);

        if (toPerform == "probe") {
            if (footInfo == "right") {
                leftLEDPos = atoi(strtok(NULL, ","));
                rightMidLEDPos = atoi(strtok(NULL, ","));
                Serial.print("Right LED info: ");
                Serial.print(leftLEDPos);
                Serial.println(rightMidLEDPos);
            } else if (footInfo == "left") {
                rightLEDPos = atoi(strtok(NULL, ","));
                leftMidLEDPos = atoi(strtok(NULL, ","));
                Serial.print("Left LED info: ");
                Serial.print(rightLEDPos);
                Serial.println(leftMidLEDPos);
            }
        }
        newData = false;

        return true;
    }

    return false;
}


byte getAppInput() {
    if (serialComm.available()>0) {
        byte inputByte = serialComm.read();

        char appData[20];
        sprintf(appData, "Data from app: %d", inputByte);
        Serial.println(appData);

        return inputByte;
    }
    return -1;
}
//** END **//

// TODO: testing, remove later
void runLocations(String foot) {
   Serial.println(foot);
   // W size 5
   for (byte k = 0; k < 4; k++) {
       for (byte i = 0; i < 2; i++) {
           probingCoords[k][i] = w5FootsizeCoords[k][i];
       }
   }

   // recalculate probing coordinates with new reference point
   for (byte i = 0; i < 4; i++) {
       for (byte j = 0; j < 2; j++) {
           probingCoords[i][j] = probingCoords[i][j] - newReferencePoint[j];
       }
   }

   Serial.println("Location Set #1 (W5)");

   for (byte a = 0; a < 4; a++) {
       if (foot == "left") {
           moveXYRails(-probingCoords[a][0], -probingCoords[a][1]); // left
       } else {
           moveXYRails(probingCoords[a][0], -probingCoords[a][1]);
       }
       
       for (int mmLocation = 1; mmLocation < actuatorMaxTravelDist; mmLocation += strokeStepMM) {
           moveActuator(mmLocation); // move actuator to mmLocation
           delay(delayMS); // delay by delayMS before next stroke
       }
       delay(100);
       moveActuator(0);
       delay(250);
   }

   resetMotors();
   delay(1000);

    for (byte j = 0; j < 9; j++) {
        for (byte i = 0; i < 2; i++) {
            probingCoords[0][i] = m1CoordList[j][i];
            probingCoords[1][i] = probingCoords[0][i]*m3Conversions[i];
            probingCoords[2][i] = probingCoords[0][i]*m5Conversions[i];
            probingCoords[3][i] = probingCoords[0][i]*bigToeConversions[i];
        }

        // recalculate probing coordinates with new reference point
        for (byte i = 0; i < 4; i++) {
            for (byte j = 0; j < 2; j++) {
                probingCoords[i][j] = probingCoords[i][j] - newReferencePoint[j];
            }
        }

        Serial.print("Location Set #");
        Serial.println(j+2);

        for (byte a = 0; a < 4; a++) {
            if (foot == "left") {
                moveXYRails(-probingCoords[a][0], -probingCoords[a][1]); // left
            } else {
                moveXYRails(probingCoords[a][0], -probingCoords[a][1]);
            }

            for (int mmLocation = 1; mmLocation < actuatorMaxTravelDist; mmLocation += strokeStepMM) {
                moveActuator(mmLocation); // move actuator to mmLocation
                delay(delayMS); // delay by delayMS before next stroke
            }
            delay(100);
            moveActuator(0);
            delay(250);
        }

        resetMotors();
        delay(1000);
    }
}


void calibratePSU(int ms) {
    moveActuator(actuatorMaxTravelDist);
    delay(ms);
    moveActuator(0);
}


void printPhotoValues() {
    int photoVal1 = analogRead(interrupterPin1); // read the value from the first photointerrupter
    int photoVal2 = analogRead(interrupterPin2); // read the value from the second photointerrupter
    Serial.println(photoVal1);
    Serial.println(photoVal2);
}


void setup() {
    // setting up bluetooth 
    pinMode(rxBluetoothPin, INPUT);
    pinMode(txBluetoothPin, OUTPUT);
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

    resetLEDs();
    
    // TODO: testing, remove later
    calibratePSU(2000);

    // TODO: testing, remove later
    runLocations("left");
    // runLocations("right");
}


void loop() {
    // TODO: testing, remove later
    //printPhotoValues();

    recvdAppInput = getAppCommand();

    if (recvdAppInput) {
        if (toPerform == "calibrate") {
            resetLEDs();
            byte verticalLEDStartPos = footsizeToLED[footsize+sex-5][0];
            byte horizLEDStartPos = footsizeToLED[footsize+sex-5][1];
            if (footInfo == "right") { // calibrate right foot
                Serial.println("Right calib");
                initializeLEDs("right", verticalLEDStartPos, horizLEDStartPos);
                calibrationLoop("right", verticalLEDStartPos, horizLEDStartPos);
            }
            if (footInfo == "left") { // calibrate left foot
                Serial.println("Left calib");
                initializeLEDs("left", verticalLEDStartPos, horizLEDStartPos);
                calibrationLoop("left", verticalLEDStartPos, horizLEDStartPos);
            }

            toPerform = "";
        } else if (toPerform == "probe") {
            getProbingCoords();

            if (footInfo == "right") { // probe right foot
                initializeLEDs("right", leftLEDPos, rightMidLEDPos);
                while (getAppInput() != 1) { // wait from confirmation from app
                    continue;
                }
                Serial.println("Right probe");
                probeLoop("right");
            }
            if (footInfo == "left") {
                initializeLEDs("left", rightLEDPos, leftMidLEDPos);
                while (getAppInput() != 1) { // wait from confirmation from app
                    continue;
                }
                Serial.println("Left probe");
                probeLoop("left");
            }

            toPerform = "";
            resetLEDs();
        }
    }
}
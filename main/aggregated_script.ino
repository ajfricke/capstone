#include <AccelStepper.h>
#include <Servo.h>
#include <SoftwareSerial.h>

bool fakeProbe = false;

// define Arduino pins
#define interrupterPin1 A0
#define interrupterPin2 A1
#define dirPin100 2 // stepper motor 100 DIR
#define stepPin100 3 // stepper motor 100 STEP
#define dirPin300 4 // stepper motor 300 DIR
#define stepPin300 5 // stepper motor 300 DIR
#define txBluetoothPin 6 // blue wire, HC-05
#define rxBluetoothPin 7 // green wire, HC-05
#define dataPin 9 // shift register
#define latchPin 10 // shift register
#define clockPin 11 // shift register
#define servoPin 13 // actuator

// stepper motor variables
#define mmPerRev 5
#define oneMicrostepStepsPerRev 200
#define microstep 0.25 // change if needed

// actuator variables
#define actuatorMaxTravelDist 36
#define actuatorShaftLengthMM 50
#define strokeStepMM 2
#define delayMS 100

// define stepper motors and actuator
#define motorInterfaceType 1 // indicates a driver for stepper motors
AccelStepper stepper100 = AccelStepper(motorInterfaceType, stepPin100, dirPin100);
AccelStepper stepper300 = AccelStepper(motorInterfaceType, stepPin300, dirPin300);
Servo myServo;

// app communication variables
SoftwareSerial serialComm = SoftwareSerial(rxBluetoothPin, txBluetoothPin);
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
byte sex; // 0 for female, 1 for male
byte footsize;
bool newData = false;
bool recvdAppInput = false;
byte response = -1;

// keeping track of time
unsigned long currMillis;
unsigned long prevMillis;

// LED mappings
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

// probing coordinate information
// TODO: incorporate bigToeModLeft into this list
float xCoordList[10][4] = {{-37.13021,-7.36082,19.43552,-37.13021},
                            {-36.47201,-5.59986,22.18894,-36.47201},
                            {-35.58878,-3.24636,25.86571,-35.58878},
                            {-34.9282,-1.48314,28.62138,-34.9282},
                            {-33.60309,2.04746,34.1369,-33.60309},
                            {-33.16397,3.22134,35.97242,-33.16397},
                            {-32.50065,4.98749,38.73103,-32.50065},
                            {-32.06223,6.16075,40.56592,-32.06223},
                            {-30.96128,9.09948,45.15875,-30.96128},
                            {-30.29724,10.86631,47.91804,-30.29724}};

float bigToeModLeft[10] = {0,0,0,0,0,-0.5,-0.5,-0.75,-1,-1.25};

float yCoordList[10][4] = {{-23.24159,-23.60614,-30.19157,12.20841},
                            {-17.70314,-17.02707,-23.88689,20.20841},
                            {-10.42023,-9.71417,-16.87887,29.20841},
                            {-3.86713,-3.1343,-10.57339,37.70841},
                            {3.41535,4.1785,-3.56548,46.20841},
                            {7.78528,8.56658,0.63968,52.20841},
                            {15.06718,15.87828,7.64649,60.70841},
                            {21.6208,22.45875,13.95257,68.70841},
                            {26.71761,27.57716,18.85756,75.20841},
                            {32.54347,33.4268,24.4633,82.20841}};

float probingCoords[4][2];


//** CALIBRATION FUNCTIONS **//
// turn off all LEDs
void resetLEDs() {
    //Serial.println("Resetting LEDs");
    digitalWrite(latchPin, LOW);
    for (byte i = 0; i < 4; i++) {
        shiftOut(dataPin, clockPin, MSBFIRST, 0);
    }
    digitalWrite(latchPin, HIGH);
}


// initalize LEDs for calibration or probing
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


// update foot size based on true LED positions
void updateFootsize(byte trueVertPos, byte trueHorizPos, byte estVertPos, byte estHorizPos) {
    byte horizIdxRange[4][2] = {{0, 1}, {2, 3}, {4, 6}, {7, 9}};
    //Serial.println("Updating footsize");

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


// main calibration function
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
// move motors and actuator back to starting positions
void resetMotors() {
    //Serial.println("Resetting to starting position.");
    moveActuator(0);
    delay(500);
    moveXYRails(0, 0);
}


// get and shuffle probing coordinates
void getProbingCoords() {
    //Serial.println("Getting probing coordinates");
    // determine the coordinates for each probing location
    probingCoords[0][0] = xCoordList[footsize+sex-5][0];
    probingCoords[1][0] = xCoordList[footsize+sex-5][1];
    probingCoords[2][0] = xCoordList[footsize+sex-5][2];
    probingCoords[3][0] = xCoordList[footsize+sex-5][3];

    probingCoords[0][1] = yCoordList[footsize+sex-5][0];
    probingCoords[1][1] = yCoordList[footsize+sex-5][1];
    probingCoords[2][1] = yCoordList[footsize+sex-5][2];
    probingCoords[3][1] = yCoordList[footsize+sex-5][3];

    // TODO: make sure shuffling is working
    // shuffle the sequence of probing locations
    if (fakeProbe == false) {
        //Serial.println("Not fake probe. Getting coords");
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
    myServo.writeMicroseconds(usec);
}


// raise actuator until photointerrupter input
bool raiseMonofilament() {
    // move actuator up to at most actuatorShaftLengthMM, checking for input from photointerrupters every strokeStepMM
    for (int mmLocation = 1; mmLocation < actuatorMaxTravelDist+4; mmLocation += strokeStepMM) {
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


// main probing function
void probeLoop(String foot, byte falseProbeList[4]) {
    bool properProbe;
    //Serial.println("Starting probing.");
    // move rails to each of the 4 probing locations
    for (byte i = 0; i < 4; i++) {
        //Serial.println("Moving XY rails to probing location.");
        // negate x-coordinate if left foot, negate y-coordinates
        if (foot == "left") {
            moveXYRails(probingCoords[i][0], -probingCoords[i][1]);
            
        } else {
            moveXYRails(-probingCoords[i][0], -probingCoords[i][1]);
        }

        // test probing location three times
        byte improperProbeCount = 0;
        for (byte j = 0; j < 3; j++) {
            if (falseProbeList[i] == 0) {
                //Serial.println("Raising monofilament.");
                properProbe = raiseMonofilament();
            } else {
                //Serial.println("Faking probe!");
                properProbe = true;
            }

            if (properProbe == false) {
                j -= 1;
                improperProbeCount += 1;
            } else {
                //Serial.println("Detected bend");

                currMillis = millis();
                prevMillis = millis();
                
                // check for user response
                response = -1;
                while((currMillis - prevMillis < 2000) && response != 1) {
                    response = getAppInput();
                    currMillis = millis();
                }
                if (response == 1) {
                    Serial.println("Got yes probe");
                    //Serial.println("Lowering monofilament.");
                    moveActuator(0); // lower monofilament back down
                    break;
                }
            }

            if (improperProbeCount == 3){
                //Serial.println("Failed.");
                serialComm.write(-1); // tell app we failed
                resetMotors();
                return;
            }

            //Serial.println("Lowering monofilament.");
            moveActuator(0); // lower monofilament back down

            currMillis = millis();
            prevMillis = millis();
            
            // wait for user response, otherwise will be marked as no
            response = -1;
            while((currMillis - prevMillis < 2000) && response != 1) {
                response = getAppInput();
                currMillis = millis();
            }
            if (response == 1) {
                Serial.println("Got yes probe.");
                break;
            }

            if (j < 2) {
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


//** APP COMMUNICATION FUNCTIONS **//
// get probing or calibration starting command from app
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


// get single byte input from app
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


//** TESTING FUNCTIONS **//
// runs through all probing locations, moving actuator up and down
// does 4 locations for one foot of one foot size, then the other foot, then repeats for the next foot size
void runLocations() {
   for (byte i = 2; i < 10; i++) {
        Serial.print("Location Set #");
        Serial.println(i+1);
        
        probingCoords[0][0] = xCoordList[i][0];
        probingCoords[1][0] = xCoordList[i][3];
        probingCoords[2][0] = xCoordList[i][1];
        probingCoords[3][0] = xCoordList[i][2];
    
        probingCoords[0][1] = yCoordList[i][0];
        probingCoords[1][1] = yCoordList[i][3];
        probingCoords[2][1] = yCoordList[i][1];
        probingCoords[3][1] = yCoordList[i][2];

        for (byte a = 0; a < 4; a++) {
            moveXYRails(probingCoords[a][0], -probingCoords[a][1]); // left
            Serial.println("Now");
            Serial.println(probingCoords[a][0]);
            Serial.println(-probingCoords[a][1]);

            moveActuator(actuatorMaxTravelDist);
            delay(8000);
            moveActuator(0);
            delay(500);
        }
        for (byte a = 0; a < 4; a++) {
            moveXYRails(-probingCoords[a][0], -probingCoords[a][1]); // right

            moveActuator(actuatorMaxTravelDist);
            delay(8000);
            moveActuator(0);
            delay(500);
        }

        resetMotors();
        moveActuator(actuatorMaxTravelDist);
        delay(5000);
        moveActuator(0);
        delay(2000);
   }  
}

// runs to all locations of the same location with actuator raised
void runLocations2(byte location) {
   for (byte i = 0; i < 10; i++) {
        probingCoords[0][0] = xCoordList[i][0];
        probingCoords[1][0] = xCoordList[i][3];//+bigToeModLeft[i];
        probingCoords[2][0] = xCoordList[i][1];
        probingCoords[3][0] = xCoordList[i][2];
    
        probingCoords[0][1] = yCoordList[i][0];
        probingCoords[1][1] = yCoordList[i][3];
        probingCoords[2][1] = yCoordList[i][1];
        probingCoords[3][1] = yCoordList[i][2];

        moveXYRails(probingCoords[location][0], -probingCoords[location][1]); // left
        moveActuator(actuatorMaxTravelDist);
        delay(3000);
   }
   resetMotors();
    moveActuator(actuatorMaxTravelDist);
    delay(5000);
    moveActuator(0);
    delay(2000);
        
   for (byte i = 0; i < 10; i++) {       
        probingCoords[0][0] = xCoordList[i][0];
        probingCoords[1][0] = xCoordList[i][3];//+bigToeModLeft[i];//
        probingCoords[2][0] = xCoordList[i][1];
        probingCoords[3][0] = xCoordList[i][2];
    
        probingCoords[0][1] = yCoordList[i][0];
        probingCoords[1][1] = yCoordList[i][3];
        probingCoords[2][1] = yCoordList[i][1];
        probingCoords[3][1] = yCoordList[i][2];

        moveXYRails(-probingCoords[location][0], -probingCoords[location][1]); // left
        moveActuator(actuatorMaxTravelDist);
        delay(3000);
   }
   resetMotors();
}


// do M1 coordinate followed by big toe for each foot size and left/right
void runLocations3() {
   for (byte i = 0; i < 10; i++) {
        probingCoords[0][0] = xCoordList[i][0];
        probingCoords[1][0] = xCoordList[i][3];
        probingCoords[2][0] = xCoordList[i][1];
        probingCoords[3][0] = xCoordList[i][2];
    
        probingCoords[0][1] = yCoordList[i][0];
        probingCoords[1][1] = yCoordList[i][3];
        probingCoords[2][1] = yCoordList[i][1];
        probingCoords[3][1] = yCoordList[i][2];

        moveXYRails(probingCoords[0][0], -probingCoords[0][1]); // left M1
        moveActuator(actuatorMaxTravelDist);
        delay(3000);
        moveActuator(actuatorMaxTravelDist-20);
        moveXYRails(probingCoords[1][0], -probingCoords[1][1]); // left big toe
        moveActuator(actuatorMaxTravelDist);
        delay(3000);
        moveActuator(actuatorMaxTravelDist-20);
   }
   resetMotors();
    moveActuator(actuatorMaxTravelDist);
    delay(5000);
    moveActuator(0);
    delay(2000);
        
   for (byte i = 0; i < 10; i++) {       
        probingCoords[0][0] = xCoordList[i][0];
        probingCoords[1][0] = xCoordList[i][3];
        probingCoords[2][0] = xCoordList[i][1];
        probingCoords[3][0] = xCoordList[i][2];
    
        probingCoords[0][1] = yCoordList[i][0];
        probingCoords[1][1] = yCoordList[i][3];
        probingCoords[2][1] = yCoordList[i][1];
        probingCoords[3][1] = yCoordList[i][2];

        moveXYRails(-probingCoords[0][0], -probingCoords[0][1]); // right M1
        moveActuator(actuatorMaxTravelDist);
        delay(3000);
        moveActuator(actuatorMaxTravelDist-20);
        moveXYRails(-probingCoords[1][0], -probingCoords[1][1]); // right big toe
        moveActuator(actuatorMaxTravelDist);
        delay(3000);
        moveActuator(actuatorMaxTravelDist-20);
   }
   resetMotors();
}


// calibrate starting location of monofilament
void calibratePSU(int ms) {
    moveActuator(actuatorMaxTravelDist);
    delay(ms);
    moveActuator(0);
}


// continually print values from the photointerrupters
void printPhotoValues() {
    int photoVal1 = analogRead(interrupterPin1); // read the value from the first photointerrupter
    int photoVal2 = analogRead(interrupterPin2); // read the value from the second photointerrupter
    Serial.println(photoVal1);
    Serial.println(photoVal2);
}


// repeatedly raise and lower monofilament, pausing when bend is detected
void bendTest() {
  for (byte i = 0; i < 20; i++) {
    bool result = raiseMonofilament();

    if (result == true) {
        delay(3000);
        moveActuator(0);
        delay(1000);
    } else {
        delay(1000);
    }
  }
}
//** END **//


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
    calibratePSU(5000);
    delay(1000);

    // TODO: testing, remove later
    //runLocations2(1);
    //runLocations3();
    //bendTest();
}


void loop() {
    recvdAppInput = getAppCommand();

    if (recvdAppInput) {
        if (toPerform == "calibrate") {
            resetLEDs();
            byte verticalLEDStartPos = footsizeToLED[footsize+sex-5][0];
            byte horizLEDStartPos = footsizeToLED[footsize+sex-5][1];
            if (footInfo == "right") { // calibrate right foot
                //Serial.println("Right calib");
                initializeLEDs("right", verticalLEDStartPos, horizLEDStartPos);
                calibrationLoop("right", verticalLEDStartPos, horizLEDStartPos);
            }
            if (footInfo == "left") { // calibrate left foot
                //Serial.println("Left calib");
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
                //Serial.println("Right probe");
                byte rightList[4] = {0,0,0,0};
                probeLoop("right", rightList); // m1, m3, m5, big toe
            }
            if (footInfo == "left") {
                initializeLEDs("left", rightLEDPos, leftMidLEDPos);
                while (getAppInput() != 1) { // wait from confirmation from app
                    continue;
                }
                //Serial.println("Left probe");
                byte leftList[4] = {0,0,0,0};
                probeLoop("left", leftList); // m1, m3, m5, big toe
            }

            toPerform = "";
            resetLEDs();
        }
    }
}
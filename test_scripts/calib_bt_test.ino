// CALIBRATION WALKTHROUGH
// To start, app should send a string formatted as follows "calibrate,foot,footsize,sex",
// where foot is left/right/both, footsize is a number, and sex is 0 for female or 1 for male
// Example: "calibrate,left,10,1"
// Sending this in, the starting LEDs should turn on. If you calibrate both feet, it"ll start with right
// Now you should be able to send commands to move the LEDs:
// for vertical LED, send 0 for down and 1 for up
// for horizontal LED, send 2 for inwards and 3 for outwards
// once LEDs are in the right place, send 4 for confirmation
// NOTE: these are ints for the commands, not a string like the starting information
// Upon confirmation, Arduino sends the vertical LED position (int), 
// then the horizontal LED position (int), to the app.
// If both was specified, calibration for left foot will begin, otherwise Arduino will go back to waiting
// In waiting mode, you can send another string of starting information to run again

#include <SoftwareSerial.h>

// for HC-05 Bluetooth Module
#define rxBluetoothPin 2 
#define txBluetoothPin 3

// for shift register
#define clockPin 9 
#define latchPin 11
#define dataPin 12

SoftwareSerial serialComm (txBluetoothPin, rxBluetoothPin);

int mode = 0; // 0 for waiting, 1 for calibration, 2 for probing

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];

String footInfo;
String toPerform;

bool newData = false;
bool recvdAppInput = false;

int sex; // 0 for female, 1 for male
int footsize;

int footsizeToLED[10][2] = {{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}, {6, 2}, {6, 3}, {7, 3}, {8, 3}};

int leftLEDs[9][4] = {{0,0,0,1}, {0,0,0,2}, {0,0,0,4}, {0,0,0,8},
					   {0,0,0,16}, {0,0,0,32}, {0,0,0,64}, {0,0,0,128},
                  	   {0,0,1,0}};

int rightLEDs[9][4] = {{0,0,4,0}, {0,0,8,0}, {0,0,16,0}, {0,0,32,0}, 
					    {0,0,64,0}, {0,0,128,0}, {0,1,0,0}, {0,2,0,0}, 
					    {0,4,0,0}};

int leftMidLEDs[4][4] = {{8,0,0,0}, {4,0,0,0}, {2,0,0,0}, {1,0,0,0}};

int rightMidLEDs[4][4] = {{16,0,0,0}, {32,0,0,0}, {64,0,0,0}, {128,0,0,0}};

int leftLEDPos = -1;
int rightLEDPos = -1;
int leftMidLEDPos = -1;
int rightMidLEDPos = -1;


//** CALIBRATION FUNCTIONS **//
void initializeLEDs(String foot, int verticalPos, int horizPos) {
    digitalWrite(latchPin, LOW);
    if (foot == "left") {
        // light up LEDs for left foot
        for (int i = 0; i < 4; i++) {
            shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[horizPos][i] + rightLEDs[verticalPos][i]);
        }
    } else if (foot == "right") {
        // light up LEDs for right foot
        for (int i = 0; i < 4; i++) {
            shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[horizPos][i] + leftLEDs[verticalPos][i]);
        }
    }
    digitalWrite(latchPin, HIGH);
}


void calibrationLoop(String foot, int verticalStartPos, int horizStartPos) {
    int verticalPos = verticalStartPos;
    int horizPos = horizStartPos;
    bool finished = false;
    bool moveLED = false;
    byte instruction = -1;

    while (finished == false) {
        Serial.println("Starting calibration.");
        instruction = getAppInput(); 

        if (instruction == -1) {
            continue;
        } else if (instruction == 0) { // move vertical LED down
            Serial.println("Moving vertical LED inwards.");
            if (verticalPos != 0) {
                verticalPos--;
            } else {
                Serial.println("Hit first LED. Move outwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 1) { // move vertical LED up
            Serial.println("Moving vertical LED outwards.");
            if (verticalPos != 8) {
                verticalPos++;
            } else {
                Serial.println("Hit last LED. Move inwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 2) { // move horizontal LED inwards
            Serial.println("Moving horizontal LED inwards.");
            if (horizPos != 0) {
                horizPos--;
            } else {
                Serial.println("Hit first LED. Move outwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 3) { // move horizontal LED outwards
            Serial.println("Moving horizontal LED outwards.");
            if (horizPos != 3) {
                horizPos--;
            } else {
                Serial.println("Hit first LED. Move outwards or confirm.");
            }
            moveLED = true;
        } else if (instruction == 4) { // rail confirmed
            Serial.println("Finished calibration. Sending info to app.");
            serialComm.write(verticalPos);
            serialComm.write(horizPos);
            if (foot == "left") {
                rightLEDPos = verticalPos;
                leftMidLEDPos = horizPos;
            } else if (foot == "right") {
                leftLEDPos = verticalPos;
                rightMidLEDPos = horizPos;
            }
            finished = true;
        }

        // move the LED
        if (moveLED) {
            digitalWrite(latchPin, LOW);
            if (foot == "left") {
                for (int i = 0; i < 4; i++) {
                    shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[horizPos][i] + rightLEDs[verticalPos][i]);
                }
            } else if (foot == "right") {
                for (int i = 0; i < 4; i++) {
                    shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[horizPos][i] + leftLEDs[verticalPos][i]);
                }
            }
            digitalWrite(latchPin, HIGH);
            moveLED = false;
        }
    }  
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
        Serial.println("Got info on what to perform from app: ");
        Serial.print(toPerform);

        footInfo = strtok(NULL, ","); // left, right, or both
        Serial.println("Got foot info from app: ");
        Serial.print(footInfo);

        if (toPerform == "probe") {
            if (footInfo == "right" || footInfo == "both") {
                rightLEDPos = int(strtok(NULL, ","));
                rightMidLEDPos = int(strtok(NULL, ","));
            }
            if (footInfo == "left" || footInfo == "both") {
                leftLEDPos = int(strtok(NULL, ","));
                leftMidLEDPos = int(strtok(NULL, ","));
            }
        } else if (toPerform == "calibration") {
            footsize = int(strtok(NULL, ","));
            Serial.println("Got foot size from app: ");
            Serial.print(footsize);

            sex = int(strtok(NULL, ","));
            Serial.println("Got sex from app: ");
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
        Serial.println("Received data from the app: ");
        Serial.print(inputByte);

        return inputByte;
    }
    return -1;
}
//** END **//


void setup() {
    // setting up bluetooth module
    pinMode(rxBluetoothPin, INPUT);
    pinMode(txBluetoothPin, OUTPUT);
    serialComm.begin(9600);
    Serial.begin(9600);

    // set up shift registers
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
}


void loop() {
    Serial.println("BEGIN");
    recvdAppInput = getInitialAppInput();

    if (recvdAppInput) {
        if (toPerform == "calibrate") {
            mode = 1;
        }
    }

    if (mode == 0) { // waiting for app instructions
        return;
    } else if (mode == 1) { // calibration
        int verticalLEDStartPos = footsizeToLED[footsize+sex-5][0];
        int horizLEDStartPos = footsizeToLED[footsize+sex-5][1];

        if (footInfo == "both" || footInfo == "right") { // calibrate right foot
            initializeLEDs("right", verticalLEDStartPos, horizLEDStartPos);
            calibrationLoop("right", verticalLEDStartPos, horizLEDStartPos);
        }
        if (footInfo == "both" || footInfo == "left") { // calibrate left foot
            initializeLEDs("left", verticalLEDStartPos, horizLEDStartPos);
            calibrationLoop("left", verticalLEDStartPos, horizLEDStartPos);
        }

        mode = 0; // go back to waiting
    }
}

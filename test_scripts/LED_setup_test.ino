int latchPin = 10;
int clockPin = 9;
int dataPin = 12;
int buttonOut = 4;
int buttonIn = 3;
int buttonRail = 2;

int buttonStateOut;
int buttonStateIn;
int buttonStateRail;

int i = 0;
int j;
int maxPos;

int leftState = 0;
int rightState = 0;
int midState = 0;
String currState = "left";
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

void setup() {
  Serial.begin(9600);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  
  pinMode(buttonOut, INPUT);
  pinMode(buttonIn, INPUT);
  pinMode(buttonRail, INPUT);
}

void loop() {
  buttonStateOut = digitalRead(buttonOut);
  buttonStateIn = digitalRead(buttonIn);
  buttonStateRail = digitalRead(buttonRail);
  
  if (currState == "mid") {
    maxPos = 3;
  } else {
    maxPos = 9;
  }
  
  if (buttonStateOut == HIGH) {
    if (i != maxPos) {
    	i++;
    } else {
      	i = 0;
    }
    delay(150);
  }
  
  if (buttonStateIn == HIGH) {
    if (i != 0) {
    	i--;
    } else {
      	i = maxPos;
    }
    delay(150);
  }
  
  if (buttonStateRail == HIGH) {
    if (currState == "left") {
    	currState = "mid";
      	prevState = "left";
      	prevPos = i;
    } else if (currState == "right") {
		currState = "mid";
      	prevState = "right";
      	prevPos = i;
    } else if (currState == "mid") {
      if (prevState == "right") {
        currState = "left";
      } else if (prevState == "left") {
        currState = "right";
      }
    }
    i = 0;
    delay(150);
  }
  digitalWrite(latchPin, LOW);
  if (currState == "left") {
    shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[i][0]);
    shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[i][1]);
    shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[i][2]);
    shiftOut(dataPin, clockPin, MSBFIRST, leftLEDs[i][3]);
  } else if (currState == "right") {
    shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[i][0]);
    shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[i][1]);
    shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[i][2]);
    shiftOut(dataPin, clockPin, MSBFIRST, rightLEDs[i][3]);
  } else if (currState == "mid") {
    if (prevState == "left") {
      shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[i][0] + leftLEDs[prevPos][0]);
      shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[i][1] + leftLEDs[prevPos][1]);
      shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[i][2] + leftLEDs[prevPos][2]);
      shiftOut(dataPin, clockPin, MSBFIRST, leftMidLEDs[i][3] + leftLEDs[prevPos][3]);
    } else if (prevState == "right") {
      j = 3-i;
      shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[j][0] + rightLEDs[prevPos][0]);
      shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[j][1] + rightLEDs[prevPos][1]);
      shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[j][2] + rightLEDs[prevPos][2]);
      shiftOut(dataPin, clockPin, MSBFIRST, rightMidLEDs[j][3] + rightLEDs[prevPos][3]);
    }
    
  }
  digitalWrite(latchPin, HIGH);
}

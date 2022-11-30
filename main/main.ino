// Bluetooth communication
int input_bit = -1;
char input_byte = 'z';

// Arduino pin connections for the shift registers
int shiftClock = 9;
int latchClock = 10;
int serialData = 12;

// footsize to LED positions mapping
int footsize_LED_map_F[][2] = {{1,1}, {2,2}, {3,3}, {4,4}};
int footsize_LED_map_M[][2] = {{1,1}, {2,2}, {3,3}, {4,4}};

// LED position to LED pin mapping
int LED_pos_to_pin_middle[] = {13, 14, 15, 16, 17};
int LED_pos_to_pin_L[] = {13, 14, 15, 16, 17};
int LED_pos_to_pin_R[] = {13, 14, 15, 16, 17};

struct LED_pos {
  int side_pos;
  int top_pos;
  int foot;
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // set shift register pins to output
  pinMode(shiftClock, OUTPUT);
  pinMode(latchClock, OUTPUT);
  pinMode(serialData, OUTPUT);

  digitalWrite(latchClock, LOW); // put low so LEDs don't change while sending in bits
  shiftOut(serialData, shiftClock, MSBFIRST, shiftCount)
}

void loop() {
  // put your main code here, to run repeatedly:

}

void main() {
  // controls everything
  // handles signals from tablet

  // get information over BT
  // might need to change input bit variable, depending on what is being sent
  while(Serial.available()>0) {
    input_bit = Serial.read()
    Serial.println(input_bit);

    if (input_bit) == 0 {
      start_setup();
    } else if (input_bit) == 1 {
      start_test();
    }

    // footsize (int), foot type (L,R), or LED positions (ie. 111)
  }
}

// LED SETUP FUNCTIONS
void start_setup() {
  while(Serial.available()>0) {
    input_byte = Serial.read()
    Serial.println(input_byte);

    if (input_byte > 99) {
      initialize_LEDs(input_byte, false);
      return;
    } else {
      pos = footsize_to_LED_pos(input_byte, true);
      initialize_LEDs(pos);
      handle_LEDs(pos);
      return;
    }
  }
}

struct LED_pos footsize_to_LED_pos(int sex, int foot, double footsize) {
  struct LED_pos pos;

  // convert footsize to an index for the mapping
  footsize_idx = foot_size/2*3; // just an example

  // get LED locations based on footsize and sex
  if (sex == 0) {
    LED_locations = footsize_LED_map_F[footsize_LED_mapping];
  } else {
    LED_locations = footsize_LED_map_M[footsize_LED_mapping];
  }

  // use mapping to link footsize to LED positions
  pos.side_pos = mapping[0];
  pos.top_pos = mapping[1];
  pos.foot = foot;

  return pos;
}

// turn on LEDs
void initialize_LEDs(struct pos) {



  digitalWrite(LED_pos_to_pin_middle[pos.side_pos], HIGH)
  
  if (pos.foot == 0) {
    digitalWrite(LED_pos_to_pin_R[pos.top_pos], HIGH);
  } else {
    digitalWrite(LED_pos_to_pin_L[pos.top_pos], HIGH);
  }
}

// parent function to move control the movement of the LEDs
void handle_LEDs(struct pos) {
  move_LEDs(pos.side, LED_pos_to_pin_middle);

  if (pos.foot == 0) {
    move_LEDs(pos.top_pos, LED_pos_to_pin_R);
  } else {
    move_LEDs(pos.top_pos, LED_pos_to_pin_L);
  }
}


// TODO: Add checks for invalid positions (i.e. goes too high)
void move_LED(struct pos, int map[]) {
  // get information over BT
  while(Serial.available()>0) {
    input_bit = Serial.read()
    Serial.println(input_bit);

    // move inwards
    if (input_bit) == 0 {
      digitalWrite(map[pos], LOW);
      digitalWrite(map[pos-1], HIGH);
      pos = pos-1;
    } 
    // move outwards
    else if (input_bit) == 1 {
      digitalWrite(map[pos], LOW);
      digitalWrite(map[pos+1], HIGH);
      pos = pos+1;
    }
    // send position to tablet over BT, LED placement confirmed
    else (input_bit) == 2 {
      return
    }
  }
}

void start_test() {
  while(Serial.available()>0) {
    input_byte = Serial.read()
    Serial.println(input_byte);

    if (input_byte == 'L') {
      // call get_random_sequence
    } else if (input_byte == 'R') {
      // call get_random_sequence
    }
  }
}

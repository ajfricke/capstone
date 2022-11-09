// Bluetooth communication
int input_bit = -1;
char input_byte = 'z';

// example foot mapping
array footsize_LED_mapping = {{1,1,1}, {2,2,2}, {3,3,3}, {4,4,4}};

struct LED_pos {
  int left_pos;
  int right_pos;
  int top_pos;
  
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // set pin 13 as an output pin for LED (will have to do this for all LEDs)
  pinMode(13, OUTPUT);
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
      return;
    }
  }
}

struct LED_pos footsize_to_LED_pos(double footsize) {
  struct LED_pos pos;

  // convert footsize to an index for the mapping
  footsize_idx = foot_size/2*3;
  mapping = footsize_LED_mapping[footsize_LED_mapping];

  // use mapping to link footsize to LED positions
  pos.left_pos = mapping[0];
  pos.right_pos = mapping[1];
  pos.top_pos = mapping[2];

  return pos;
}

void initialize_LEDs(pos, move_LEDs_bool) {
  // turn on LEDs

  if (move_LEDs_bool == true) {
    // solid red (unnoticable PWM)
    // call handle_LEDs
    handle_LEDs(pos);
    return;
  } else {
    // solid green
    return;
  }
}

void handle_LEDs(pos) {
  // control set up
  // in future, flash LEDs rather than putting them high
  // need mapping of LED positions to pin output?
  move_LED(pos.left_pos);
  move_LED(pos.right_pos);
  move_LED(pos.top_pos);
  
  return;
}

void move_LED(pos) {
  // get information over BT
  while(Serial.available()>0) {
    input_bit = Serial.read()
    Serial.println(input_bit);

    if (input_bit) == 0 {
      // move inwards
      digitalWrite(pos, LOW);
      digitalWrite(pos-1, HIGH);
      pos = pos-1;
    } else if (input_bit) == 1 {
      // move outwards
      digitalWrite(pos, LOW);
      digitalWrite(pos+1, HIGH);
      pos = pos+1;
    } else (input_bit) == 2 {
      // send position to tablet over BT
      return // LED placement confirmed
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

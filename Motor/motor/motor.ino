class Motor {
private:
 byte dir_pin;
 byte step_pin;
 int step_count;
 bool enable;
 
public:
  Motor() {
  step_count = 0;
  dir_pin = 0;
  step_pin = 0;
  enable = false;
 }
 void init(byte dir_p, byte step_p) {
  dir_pin = dir_p;
  step_pin = step_p;
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
 }
 byte resetStepCount() {
  step_count = 0;
 }
 void rotate() {
  if (!enable) {
    return;
  }
  digitalWrite(step_pin,HIGH);
  delayMicroseconds(5000);
//  delay(50);
  digitalWrite(step_pin,LOW);
  delayMicroseconds(5000);
//  delay(50);
  step_count++;
 }
 bool clockwise() const {
  digitalWrite(dir_pin,HIGH);
 }
 void counterClockwise() const {
  digitalWrite(dir_pin,LOW);
 }
 int getStepCount() const {
  return step_count;
 }
 void enableMotor() {
    enable = true;
 }
 void disableMotor() {
    enable = false;
 }
};

Motor motor1;
Motor motor2;
Motor motor3;
Motor motor4;

const byte button_pin = 11;
const byte led_pin = 10;

bool motor_run = false;
int reading;
int previous = LOW;
long time = 0; 
long debounce = 200;

int FULL_REV = 200;

// for Serial com
String inputString = ""; 

void setup() { 
  Serial.begin(9600);

  motor1.init(3,2);
  motor2.init(5,4);
  motor3.init(7,6);
  motor4.init(9,8);
  
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
}

void loop() {
  reading = digitalRead(button_pin);
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (motor_run) {
      motor_run = false;
      digitalWrite(led_pin, LOW);
    }
    else {
      digitalWrite(led_pin, HIGH);
      motor_run = true;
    }
    
    motor1.resetStepCount();
    motor2.resetStepCount();
    motor3.resetStepCount();
    motor4.resetStepCount();
  }
  
  previous = reading;
  
  if (motor_run) {
    motor1.rotate();
    motor2.rotate();
    motor3.rotate();
    motor4.rotate();
  }
}

void checkString () {
  if (inputString == "e run") // elbow motor
    motor1.enableMotor();
  else if (inputString == "s run") // shoulder motor
    motor2.enableMotor();
  else if (inputString == "g run") // gripper motor
    motor3.enableMotor();
  else if (inputString == "t run") // twister motor
    motor4.enableMotor();
  else if (inputString == "e stop") // elbow motor
    motor1.disableMotor();
  else if (inputString == "s stop") // shoulder motor
    motor2.disableMotor();
  else if (inputString == "g stop") // gripper motor
    motor3.disableMotor();
  else if (inputString == "t stop") // twister motor
    motor4.disableMotor();
  else{
    Serial.print("ERROR: Unknown command ");
    Serial.println(inputString);
  }
    
}

// occur between each loop
void serialEvent() {
  char inChar;
  while (Serial.available() > 0) {
    // get the new byte:
    inChar = (char)Serial.read();
    // end of command
    if (inChar == '\n') {
      checkString();
      inputString = "";
    } else {
      // add it to the inputString:
      inputString += inChar;
    }
  }
}

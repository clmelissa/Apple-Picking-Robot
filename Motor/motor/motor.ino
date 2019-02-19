class Motor {
private:
 byte dir_pin;
 byte step_pin;
 int step_count;
 
public:
  Motor() {
  step_count = 0;
  dir_pin = 0;
  step_pin = 0;
 }
 void init(byte dir_p, byte step_p) {
  step_count = 0;
  dir_pin = dir_p;
  step_pin = step_p;
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
 }
 byte resetStepCount() {
  step_count = 0;
 }
 void rotate() {
  digitalWrite(step_pin,HIGH);
  delayMicroseconds(3000);
  digitalWrite(step_pin,LOW);
  delayMicroseconds(3000);
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
    if (motor_run)
      motor_run = false;
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
    if (motor1.getStepCount() == FULL_REV) {
      motor_run = false;
      digitalWrite(led_pin, LOW);
    }
  }
  
}

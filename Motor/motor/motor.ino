#include "motor.h"
//#include "kinematics.h"

/*****************************************************************************************/
/****************************************MOTOR********************************************/
/*****************************************************************************************/
//LinkMotor s_motor;
//LinkMotor e_motor(2, 1.1);
Motor s_motor;
Motor e_motor;
VerticalMotor v_motor(32,2.5);
Motor t_motor;
Motor g_motor;

int target_angle = 0;

int s_speed = 50;
int e_speed = 90;

bool motor_run = false;
bool return_to_zero_pos = false;

/*****************************************************************************************/
/***************************************SENSORS*******************************************/
/*****************************************************************************************/
// Force Resistor
int fsrAnalogPin = 1; 
// Buttons and LED
const byte green_led =  39;
const byte green_button = 5;
const byte red_led = 37;
const byte red_button = 10; 
const byte yellow_led = 6;
const byte r_button = 2;
const byte l_button = 3;
int previous = LOW;
long time = 0; 
long debounce = 200;
int reading;

bool applePicked; 
const int numReads = 10;

// for Serial com
String inputString = ""; 

// Potentiometer
int shoulder_pot = A0;
int elbow_pot = A1;
float val = 0;
float t = 0;
float v;
float R1 = 10000;
float R2;
float ang;
float R2_counter;
float counter;
float R2_av;

bool elbow_target = false;
float e_target;
bool shoulder_target = false;
float s_target;

float s_ang;
float e_ang;

bool vertical_target = false;
int v_steps = 0;

/*****************************************************************************************/
/*************************************FUNCTIONS*******************************************/
/*****************************************************************************************/

bool pressDetected(){
  int total = 0;
  bool result = false;
  for(int i = 0; i< numReads; i++){
//    Serial.println(analogRead(fsrAnalogPin));
    total += analogRead(fsrAnalogPin);
  }
  
  if(total/numReads > 10){
    result = true;
  }
  
  return result;
 
}

float getAngle(int analogPin) {
//  float sum = 0;
//  for(int i = 0; i < points; i++){
//    sum = sum + R1/(1023.0/analogRead(analogPin) - 1)/1000000;
//    delay(d);
//  }
  val = analogRead(analogPin);
  float y_intercept = 172.0;
  if (analogPin == elbow_pot)
    y_intercept = 131.0;
  
  ang = -(18.0/70.0)*val + 90.0 + (18.0/70.0)*y_intercept;
  if (analogPin == elbow_pot)
    ang+=10.5;
  else
    ang-=10.5;
  return ang;
}

/*****************************************************************************************/
/******************************************MAIN*******************************************/
/*****************************************************************************************/

void setup() { 
  Serial.begin(9600);

  // Shoulder
  s_motor.init(53,52);
  // Elbow
  e_motor.init(42,41); 
  // Vertical
  v_motor.init(49,47);
  // Gripper
  g_motor.init(48,46);
  // Twister
  t_motor.init(24,22);  
  
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
//  pinMode(yellow_led, OUTPUT);
  
  pinMode(r_button, INPUT);
  pinMode(l_button, INPUT);
  pinMode(red_button, INPUT);
//  pinMode(green_button, INPUT);

  Serial.println("Set up done");
}

void loop() {
  reading = digitalRead(red_button);
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (motor_run) {
      motor_run = false;
      digitalWrite(green_led, LOW);
    }
    else {
      digitalWrite(green_led, HIGH);
      motor_run = true;
    }
    time = millis();
  }
  previous = reading;

//  // Check if elbow hits the limit
//  reading = digitalRead(r_button);
//  
//  if (reading == HIGH) {
//    // turn LED on:
//    digitalWrite(red_led, HIGH);
//    e_motor.disableMotor();
//  } else {
//    // turn LED off:
//    reading = digitalRead(l_button);
//    if (reading == HIGH) {
//      digitalWrite(red_led, HIGH);
//      e_motor.disableMotor();
//    } else {
//      // turn LED off:
//      digitalWrite(red_led, LOW);
//    }
//  }
//  
  if (motor_run) {
    s_motor.rotate(s_speed);
    e_motor.rotate(e_speed);
    v_motor.rotate(1);
    g_motor.rotate(5);
    t_motor.rotate(3); 
//    if (pressDetected()) {
//      t_motor.disableMotor();
//    }  
    if (elbow_target) {
      e_ang = getAngle(elbow_pot);
      if (abs(e_ang - e_target) < 3) {
        Serial.print("INFO: Arrive at angle ");
        Serial.println(e_target);
        elbow_target = false;
        e_motor.disableMotor();
        e_target = 0;
      }
      
      else if (e_target < e_ang && !e_motor.isClockwise()) {
//        Serial.print("clockwise");
        e_motor.clockwise();
      } else if (e_target > e_ang && e_motor.isClockwise()) {
//        Serial.print("counterClockwise");
        e_motor.counterClockwise();
      }
    }
    if (shoulder_target) {
      s_ang = getAngle(shoulder_pot);
      if (abs(s_ang - s_target) < 3) {
//        Serial.print("INFO: Arrive at angle ");
//        Serial.println(s_target);
        shoulder_target = false;
        s_motor.disableMotor();
        s_target = 0;
        Serial.print("g");
      }
      
      else if (s_target < s_ang && !s_motor.isClockwise()) {
        s_motor.clockwise();
      } else if (s_target > s_ang && s_motor.isClockwise()) {
        s_motor.counterClockwise();
      }
    }
    if (vertical_target) {
      v_steps--;
      if (v_steps == 0) {
        vertical_target = false;
        v_motor.disableMotor();
      }
    }    
  }
//  Serial.print("Press = ");
//  Serial.println(applePicked);
}



/*****************************************************************************************/
/*********************************SERIAL ARGUMENTS****************************************/
/*****************************************************************************************/

void checkString () {
//  Serial.print("INFO: Serial command : ");
////  Serial.println(inputString);
  if (inputString == "s run") // shoulder motor
    s_motor.enableMotor();
  else if (inputString == "s stop") // shoulder motor
    s_motor.disableMotor();
  else if (inputString == "s cw")
    s_motor.clockwise();
  else if (inputString == "s ccw")
    s_motor.counterClockwise();
  
  else if (inputString.substring(0,7) == "starget") {
    s_target = inputString.substring(8).toFloat();
    Serial.print("Moving the shoulder to ");
    Serial.println(s_target);
    s_ang = getAngle(shoulder_pot);
    Serial.print("Curent shoulder angle is ");
    Serial.println(s_ang);
    shoulder_target = true;
    if (s_target<s_ang)
      s_motor.clockwise();
    else 
      s_motor.counterClockwise();
    s_motor.enableMotor();
  }
  
  else if (inputString == "s ang") {
    s_ang = getAngle(shoulder_pot);
    Serial.print("Curent shoulder angle is ");
    Serial.println(s_ang);
  }
    
  else if (inputString == "e run") // elbow motor
    e_motor.enableMotor();
  else if (inputString == "e stop") // elbow motor
    e_motor.disableMotor();
  else if (inputString == "e cw")
    e_motor.clockwise();
  else if (inputString == "e ccw")
    e_motor.counterClockwise();
  else if (inputString.substring(0,7) == "etarget") {
    e_target = inputString.substring(8).toFloat();
//    Serial.print("Moving the elbow to ");
    Serial.println(e_target);
    e_ang = getAngle(elbow_pot);
//    Serial.print("Curent elbow angle is ");
    Serial.println(e_ang);
    elbow_target = true;
    if (e_target<e_ang){
//      Serial.print("Elbow motor is  moving CW");
      e_motor.clockwise();}
    else {
//      Serial.print("Elbow motor is  moving CCW");
      e_motor.counterClockwise();}
    e_motor.enableMotor();
  }
  
  else if (inputString == "e ang") {
    e_ang = getAngle(elbow_pot);
    Serial.print("Curent elbow angle is ");
    Serial.println(e_ang);
  }

  else if (inputString == "v run") // vertical motor
    v_motor.enableMotor();

  else if (inputString == "v stop") // vertical motor
    v_motor.disableMotor();
  else if (inputString == "v cw")
    v_motor.clockwise();
  else if (inputString == "v ccw")
    v_motor.counterClockwise();
    
  else if (inputString == "g run") // gripper motor
    g_motor.enableMotor();
  else if (inputString == "g stop") // gripper motor
    g_motor.disableMotor();  
  else if (inputString == "g cw")
    g_motor.clockwise();
  else if (inputString == "g ccw")
    g_motor.counterClockwise();
  // Number of steps to grip is 760-800
  else if (inputString == "g step") {
    Serial.print("Number of steps of gripper motor:");
    Serial.println(g_motor.getPosition());
  }


  else if (inputString == "t run") // twister motor
    t_motor.enableMotor();
  else if (inputString == "t stop") // twister motor
    t_motor.disableMotor();
  else if (inputString == "t cw")
    t_motor.clockwise();
  else if (inputString == "t ccw")
    t_motor.counterClockwise();
  else if (inputString == "t step") {
    Serial.print("Number of steps of twister motor:");
    Serial.println(t_motor.getPosition());
  }
//  else if (inputString == "g check") {
//    
//    g_motor.enableMotor();
//    g_motor.clockwise();
//    for (int i = 0; i < 790; i++) {
//      g_motor.rotate(5);
//    }
//    g_motor.disableMotor();
//  }

  else if (inputString == "s pos") {
    Serial.print("Number of steps of shoulder motor:");
    Serial.println(s_motor.getPosition());
  }
  else if (inputString == "e pos") {
    Serial.print("Number of steps of elbow motor:");
    Serial.println(e_motor.getPosition());
  }
  
  else if (inputString == "v pos") {
    Serial.print("Number of steps of vertical motor:");
    Serial.println(v_motor.getPosition());
  }

  else if (inputString == "grip") {
    g_motor.counterClockwise();
    g_motor.enableMotor();
    for (int i = 0; i < 660; i++) {
      g_motor.rotate(5);
    }
    g_motor.disableMotor();
    Serial.println("g");
  }
  else if (inputString == "release") {
    g_motor.clockwise();
    g_motor.enableMotor();
    for (int i = 0; i < 660; i++) {
      e_motor.rotate(5);
    }
    g_motor.disableMotor();
    Serial.println("g");
  }

  // Autonomous
  else if (inputString == "start") {
    delay(100);
    Serial.println("run");
//    motor_run = true;
  }
  else if (inputString == "switch") {
    delay(100);
    motor_run = true;
//    Serial.println("g");
  }
  // Change motor speed
  else if (inputString.substring(0,4) == "s sp") {
    s_speed = inputString.substring(5).toInt();
    delay(10);
  }
  else if (inputString.substring(0,4) == "e sp") {
    e_speed = inputString.substring(5).toInt();
    delay(10);
  }
  else if (inputString == "end") {
    e_motor.enableMotor();
    for (int i = 0; i < 3; i++) {
      e_motor.rotate(110);
    }
    e_motor.disableMotor();
  }
  else if (inputString.substring(0,1) == "u") {
    v_steps = inputString.substring(2).toInt();
    vertical_target = true;
    Serial.println(v_steps);
    v_motor.clockwise();
    v_motor.enableMotor();
  }
  else if (inputString.substring(0,1) == "d") {
    v_steps = inputString.substring(2).toInt();
    vertical_target = true;
    Serial.println(v_steps);
    v_motor.enableMotor();
    v_motor.counterClockwise();
  }

//  else if (inputString.substring(0,1) == "r") {
//    int num_steps = inputString.substring(2).toInt();
//    Serial.println(num_steps);
////    Serial.println("g");
//  }
//  else if (inputString.substring(0,1) == "l") {
//    int num_steps = inputString.substring(2).toInt();
//    Serial.println(num_steps);
////    Serial.println("g");
//  }
  else{
    Serial.print("wrong command ");
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

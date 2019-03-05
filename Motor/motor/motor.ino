#include "motor.h"

Motor motor1;
Motor motor2;
Motor motor3;
Motor motor4;

const byte button_pin = 11;
const byte led_pin = 10;

bool motor_run = false;
bool return_to_zero_pos = false;
int reading;
int previous = LOW;
long time = 0; 
long debounce = 200;

int FULL_REV = 200;

// for Serial com
String inputString = ""; 

void setup() { 
  Serial.begin(9600);
 
  motor1.init(7,6); 
  motor2.init(9,8);
  motor3.init(3,2);
  motor4.init(5,4);
  
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
//  Serial.print("INFO: Serial command : ");
//  Serial.println(inputString);
  if (inputString == "s run") // shoulder motor
    motor1.enableMotor();
  else if (inputString == "e run") // elbow motor
    motor2.enableMotor();
  else if (inputString == "v run") // vertical motor
    motor3.enableMotor();
  else if (inputString == "t run") // twister motor
    motor4.enableMotor();

  else if (inputString == "s stop") // shoulder motor
    motor1.disableMotor();
  else if (inputString == "e stop") // elbow motor
    motor2.disableMotor();
  else if (inputString == "v stop") // vertical motor
    motor3.disableMotor();
  else if (inputString == "t stop") // twister motor
    motor4.disableMotor();

  else if (inputString == "s cw")
    motor1.clockwise();
  else if (inputString == "s ccw")
    motor1.counterClockwise();
  else if (inputString == "s ret") {
    motor1.returnToZeroPos();
    motor1.disableMotor();
  }
  else if (inputString == "e cw")
    motor2.clockwise();
  else if (inputString == "e ccw")
    motor2.counterClockwise();
  else if (inputString == "e ret") {
    motor2.returnToZeroPos();
    motor2.disableMotor();
  }
  
  else if (inputString == "v cw")
    motor3.clockwise();
  else if (inputString == "v ccw")
    motor3.counterClockwise();
  else if (inputString == "v ret") {
    motor3.returnToZeroPos();
    motor3.disableMotor();
  }

//  else if (inputString == "s pos") {
//    Serial.print("Number of steps of shoulder motor:");
//    Serial.println(motor1.getPosition());
//  }
//  else if (inputString == "e pos") {
//    Serial.print("Number of steps of elbow motor:");
//    Serial.println(motor2.getPosition());
//  }
//  else if (inputString == "v pos") {
//    Serial.print("Number of steps of vertical motor:");
//    Serial.println(motor1.getPosition());
//  }
//  else if (inputString == "t pos") {
//    Serial.print("Number of steps of twister motor:");
//    Serial.println(motor2.getPosition());
//  }
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

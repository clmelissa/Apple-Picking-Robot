#ifndef motor_h
#define motor_h

#include <stdlib.h>
#include "Arduino.h"

const int MAX_POS = 200;

class Motor {
private:
  byte dir_pin;
  byte step_pin;
  bool enable;
  int position;
  bool CW;
 
public:
  // default constructor
  Motor() {
    dir_pin = 0;
    step_pin = 0;
    enable = false;
    position = 0;
    CW = true;
  }

  // init the direction pin and step pin
  // have to call this function before using the class
  void init(byte dir_p, byte step_p) {
    dir_pin = dir_p;
    step_pin = step_p;
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);
    clockwise();
  }

  // rotate the stepper motor if the motor is enabled
  void rotate() {
    if (enable && abs(position) < MAX_POS) {
      digitalWrite(step_pin,HIGH);
      delay(50);
      digitalWrite(step_pin,LOW);
      delay(50);
      if (CW) {
        position++;
      } else {
        position--;
      }
    }
  }

  // change motor direction to clockwise
  bool clockwise() {
    digitalWrite(dir_pin,HIGH);
    CW = true;
  }

  // change motor direction to counter clockwise
  void counterClockwise() {
    digitalWrite(dir_pin,LOW);
    CW = false;
  }

  // return current position
  int getPosition() const {
    return position;
  }

  // move back to zero position
  void returnToZeroPos() {
    if (position<0) {
      clockwise();
    } else {
      counterClockwise();
    }
    while (position!=0) {
      rotate();
    }
  }

  void enableMotor() {
    enable = true;
  }
  void disableMotor() {
    enable = false;
 }
};

#endif
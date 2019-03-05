#ifndef motor_h
#define motor_h

#include <stdlib.h>
#include "Arduino.h"


class Motor {
private:
  // Arduino pin
  byte dir_pin;
  byte step_pin;

  // if enable is false then motor won't run even if rotate() is called
  bool enable;
  // indicate the number of steps between the motor position and the initial position
  int pos;
  // indicate motor direction
  bool CW;
  // limit for the number of steps can the motor move to 1 direction
  // default is 70
  int limit;

  int target;
 
public:
  // default constructor
  Motor() {
    dir_pin = 0;
    step_pin = 0;
    enable = false;
    pos = 0;
    CW = true;
    target = -1;
  }

  // init the direction pin and step pin
  // have to call this function before using the class
  void init(byte dir_p, byte step_p, int lim = 50) {
    dir_pin = dir_p;
    step_pin = step_p;
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);
    clockwise();
    limit = lim;
  }

  // return current position
  int getPosition() const {
    return pos;
  }

  bool isClockwise() const {
    return CW;
  }

  // rotate the stepper motor if the motor is enabled
  // the lower the speed argument the faster it is
  void rotate(float motor_speed = 50) {
    // notAtLimit true if position is less than limit or the direction is going
    // will reduce the position
    bool notAtLimit = ((abs(pos) <= limit) || ((CW && pos<0)||(!CW && pos>0)));
    if (enable && notAtLimit) {
      // if moving to certain target, slows down when it gets close
      if (target != -1 && abs(pos-target)<10) {
        motor_speed+=(5*(10-abs(pos-target)));
      }
      
      digitalWrite(step_pin,HIGH);
      if (motor_speed<1)
        delay(motor_speed);
      else
        delayMicroseconds(motor_speed*1000);
      digitalWrite(step_pin,LOW);
      if (motor_speed<1)
        delay(motor_speed);
      else
        delayMicroseconds(motor_speed*1000);

      if (CW) {
        pos++;
      } else {
        pos--;
      }

      // reset target to -1 if it has reach target
      if (pos == target) {
        target = -1;
      }
    }
  }

  int getTarget() const {
    return target;
  }

  void setCurrentAsZeroPos() {
    pos=0;
  }

  // change motor direction to clockwise
  bool clockwise() {
    digitalWrite(dir_pin,LOW);
    CW = true;
    delay(500);
  }

  // change motor direction to counter clockwise
  void counterClockwise() {
    digitalWrite(dir_pin,HIGH);
    CW = false;
    delay(500);
  }

  // move back to zero position
  void returnToZeroPos() {
    if (pos<0) {
      clockwise();
    } else {
      counterClockwise();
    }
    target = 0;
  }

  void enableMotor() {
    enable = true;
  }
  void disableMotor() {
    enable = false;
 }
};

#endif

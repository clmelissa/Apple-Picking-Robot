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
  int position;
  // indicate motor direction
  bool CW;
  // limit for the number of steps can the motor move to 1 direction
  // default is 70
  int limit;
 
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
  void init(byte dir_p, byte step_p, int lim = 70) {
    dir_pin = dir_p;
    step_pin = step_p;
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);
    clockwise();
    limit = lim;
  }

  // return current position
  int getPosition() const {
    return position;
  }

  bool isClockwise() const {
    return CW;
  }

  // rotate the stepper motor if the motor is enabled
  // the lower the speed argument the faster it is
  void rotate(int speed = 50) {
    if (enable && abs(position) < limit) {
      digitalWrite(step_pin,HIGH);
      delay(speed);
      digitalWrite(step_pin,LOW);
      delay(speed);
      if (CW) {
        position++;
      } else {
        position--;
      }
    }
  }

  void setCurrentAsZeroPos() {
    position=0;
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
    if (position<0) {
      clockwise();
    } else {
      counterClockwise();
    }
    int speed = 50;
    while (position!=0) {
      // when its close to destination slows down the motor
      if (abs(position)<10) {
        speed+=5;
      }
      rotate(speed);
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
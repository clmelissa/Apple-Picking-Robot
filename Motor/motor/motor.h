#ifndef motor_h
#define motor_h

#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Arduino.h"

class Motor {
protected:
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

  bool go_to_target;
  int target;
 
public:
  // default constructor
  Motor()
  : dir_pin(0),
    step_pin(0),
    enable(false),
    pos(0),
    CW(true),
    target(-1),
    go_to_target(false) {}

  // init the direction pin and step pin
  // have to call this function before using the class
  virtual void init(byte dir_p, byte step_p, int lim = 50) {
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
  virtual void rotate(float motor_speed = 50) {
    // notAtLimit true if position is less than limit or the direction is going
    // will reduce the position
    // bool notAtLimit = ((abs(pos) <= limit) || ((CW && pos<0)||(!CW && pos>0)));

    if (enable) {
      // if moving to certain target, slows down when it gets close
      if (target != -1 && abs(pos-target)<10) {
        motor_speed+=(5*(10-abs(pos-target)));
      }
      
      digitalWrite(step_pin,HIGH);
      if (motor_speed>=1)
        delay(motor_speed);
      else
        delayMicroseconds(motor_speed*1000);
      digitalWrite(step_pin,LOW);
      if (motor_speed>=1)
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

  virtual int setTarget(int t) {
    target = t;
    go_to_target = true;
  }

  bool targetMode() const {
    return go_to_target;
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

// Link motor is for elbow and shoulder motor
// this class will have an IMU to control the movement
class LinkMotor :  public Motor {
private:
  Adafruit_BNO055* imu;
  int angle_counter;
  int angle_tolerance;

public:
  LinkMotor(Adafruit_BNO055* i, int tolerance = 5)
  : Motor(),
    imu(i),
    angle_counter(0),
    angle_tolerance(tolerance) {
  }

  virtual int setTarget(int t) {
    target = t;
    go_to_target = true;
    if (target >= 270)
      target -= 360;

    sensors_event_t event1;
    imu->getEvent(&event1);
    int curr_angle = event1.orientation.x;

    angle_counter = abs(curr_angle - target) - 5; 
    if (target < curr_angle) {
      counterClockwise();
    } else {
      clockwise();
    }
  }

  virtual void rotate(float motor_speed = 50) {
    // notAtLimit true if position is less than limit or the direction is going
    // will reduce the position
    // bool notAtLimit = ((abs(pos) <= limit) || ((CW && pos<0)||(!CW && pos>0)));

    if (enable) {
      // increase speed here because we will start checking for IMU
      if (go_to_target && angle_counter <= 0) {
        motor_speed = 0.01;

        // check curr angle
        sensors_event_t event1;
        imu->getEvent(&event1);
        int curr_angle = event1.orientation.x;
        if (curr_angle >= 270)
          curr_angle -= 360;
        if (abs(curr_angle - target) < angle_tolerance) {
          Serial.print("INFO: Arrive at angle ");
          Serial.println(target);
          go_to_target = false;
          target = 0;
          return;
        }
        else if (target < curr_angle) {
          counterClockwise();
        } else {
          clockwise();
        }
        angle_counter--;
      }

      digitalWrite(step_pin,HIGH);
      if (motor_speed>=1)
        delay(motor_speed);
      else
        delayMicroseconds(motor_speed*1000);
      
      digitalWrite(step_pin,LOW);

      if (motor_speed>=1)
        delay(motor_speed);
      else
        delayMicroseconds(motor_speed*1000);

      if (CW) {
        pos++;
      } else {
        pos--;
      }
    }
  }

}; 

#endif

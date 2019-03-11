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

 bool isEnable() const {
    return enable;
 }
};

// Link motor is for elbow and shoulder motor
// this class will have an IMU to control the movement
class LinkMotor :  public Motor {
private:
  Adafruit_BNO055 imu;
  int angle_counter;
  int angle_tolerance;
  int counter_divider;

public:
//  LinkMotor(Adafruit_BNO055* i, int tolerance = 2, int divider = 1)
//  : Motor(),
//    imu(i),
//    angle_counter(0),
//    angle_tolerance(tolerance),
//    counter_divider(divider) {
//  }
  LinkMotor(int tolerance = 2, int divider = 1)
  : Motor(),
    angle_counter(0),
    angle_tolerance(tolerance),
    counter_divider(divider) {
  }

//  void set_imu(bool shoulder) {
//    
//    if (shoulder) {
//      imu = Adafruit_BNO055(55, BNO055_ADDRESS_A);
//    } else
//      imu = Adafruit_BNO055(56, BNO055_ADDRESS_B);
//
//    if(!imu.begin())
//    {
//      /* There was a problem detecting the BNO055 ... check your connections */
//      Serial.print("Ooops, no BNO055 for elbow detected ... Check your wiring or I2C ADDR!");
//      while(1);
//    }
//    delay(10);
//      
//    imu.setExtCrystalUse(true);
//  }

  virtual int setTarget(int t) {
    target = t;
    go_to_target = true;
    if (target >= 270)
      target -= 360;

    sensors_event_t event1;
    imu.getEvent(&event1);
    int curr_angle = event1.orientation.x;
    if (curr_angle >= 270)
      curr_angle -= 360;

    Serial.print("INFO: Current angle ");
    Serial.println(curr_angle);

    angle_counter = abs(curr_angle - target)/counter_divider; 
    if (target < curr_angle) {
      counterClockwise();
    } else {
      clockwise();
    }
  }

  float getAngle() {
    sensors_event_t event1;
    imu.getEvent(&event1);
    int curr_angle = event1.orientation.x;
    if (curr_angle >= 270)
      curr_angle -= 360;
    return curr_angle;
  }

  virtual void rotate(float motor_speed = 50) {
    // notAtLimit true if position is less than limit or the direction is going
    // will reduce the position
    // bool notAtLimit = ((abs(pos) <= limit) || ((CW && pos<0)||(!CW && pos>0)));

    if (enable && go_to_target) {
      // increase speed here because we will start checking for IMU
      if (go_to_target && angle_counter<=0) {
        motor_speed = 0.005;

        // check curr angle
        sensors_event_t event1;
        imu.getEvent(&event1);
        int curr_angle = event1.orientation.x;
        if (curr_angle >= 270)
          curr_angle -= 360;
        
        Serial.print("INFO: Angle ");
        Serial.println(curr_angle);
        if (abs(curr_angle - target) < angle_tolerance) {
          Serial.print("INFO: Arrive at angle ");
          Serial.println(target);
          go_to_target = false;
          enable = false;
          target = 0;
          return;
        }
        
        else if (target < curr_angle) {
          counterClockwise();
        } else {
          clockwise();
        }
      }
      angle_counter--;

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

// Link motor is for elbow and shoulder motor
// this class will have an IMU to control the movement
class VerticalMotor : public Motor{
private:
  int ir_pin;
  int us_pin;
  double tolerance;
  double dest;

public:
  VerticalMotor(byte ir, double tol = 0.75)
  : Motor(),
    ir_pin(ir),
    us_pin(32),
    tolerance(tol),
    dest(0) {
  }

  // 0 should be base
  // min is 10 cm
  double verticalPos() {
    int val = 0;
    for (int i = 0; i < 5; i++) {
      val += analogRead(ir_pin);       // reads the value of the sharp sensor
    }
    val = val/5;
    double distance = 12343.85 * pow(val,-1.15);
    return distance;
  }

  double verticalPosSonar() {
    // establish variables for duration of the ping, and the distance result
    // in inches and centimeters:
    long duration;

    pinMode(us_pin, OUTPUT);
    digitalWrite(us_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(us_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(us_pin, LOW);

    // The same pin is used to read the signal from the PING))): a HIGH pulse
    // whose duration is the time (in microseconds) from the sending of the ping
    // to the reception of its echo off of an object.
    pinMode(us_pin, INPUT);
    duration = pulseIn(us_pin, HIGH);
    double distance = (duration) / 29.0 / 2.0;
    return distance;
  }

  virtual int setTarget(double t) {
    double current_position = verticalPos();
    dest = t;
    go_to_target = true;
    Serial.print("INFO: Current ");
    Serial.println(current_position);

    // TODO: Have to double check this 
    if (dest > verticalPos()) {
      counterClockwise();
    } else {
      clockwise();
    }
  }

}; 

#endif

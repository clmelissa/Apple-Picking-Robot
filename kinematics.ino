
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, BNO055_ADDRESS_A);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, BNO055_ADDRESS_B);


void setup() {
  // put your setup code here, to run once:

}

float xe_c;
float ye_c;

void getPosition(float &xe_c, float &ye_c, Adafruit_BNO055 bno1, Adafruit_BNO055 bno2, sensors_event_t event1, sensors_event_t event2) {

  float L1 = 0.4;
  float L2 = 0.4;
  float th1;
  float th2;
  
  bno1.getEvent(&event1);
  bno2.getEvent(&event2);

  th1 = event1.orientation.x;
  th2 = event2.orientation.x;

  xe_c = L1*sin(th1) + L2*sin(th2);
  ye_c = L1*cos(th1) + L2*cos(th2);
  
  
}


void invKin(float &th1d, float &th2d, float xed, float yed) {

  float L1 = 0.4;
  float L2 = 0.4;
  
  th1d = atan2(yed,xed) - acos((pow(xed,2)+pow(yed,2)+pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(xed,2)+pow(yed,2))));

  if ( th1d > 3.141592/2 || th1d < -3.141592/2) {
    th1d = atan2(yed,xed) + acos((pow(xed,2)+pow(yed,2)+pow(L1,2)-pow(L2,2))/(2*L1*sqrt(pow(xed,2)+pow(yed,2))));
  }
  th2d = acos( (pow(xed,2) + pow(yed,2) - pow(L1,2) - pow(L2,2) )/ (2*L1*L2) ); 
}

void loop() {
  // put your main code here, to run repeatedly:

}

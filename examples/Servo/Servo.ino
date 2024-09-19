#include <Arduino.h>
#include "SF_Servo.h"
#include "Wire.h"

SF_Servo servos = SF_Servo(Wire);

void setup(){
  Wire.begin(1, 2, 400000UL);
  servos.init();

}


void loop() {
  // put your main code here, to run repeatedly:
  setAngle(0, 128);
  setAngle(1, 256);

}
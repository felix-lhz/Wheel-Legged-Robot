#include <Arduino.h>
#include "SF_IMU.h"
#include "Wire.h"

SF_IMU mpu6050 = SF_IMU(Wire);

void setup(){
  Wire.begin(1, 2, 400000UL);
  mpu6050.init();
  mpu6050.update();

}


void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  float pitch = mpu6050.angle[0];
  float roll = mpu6050.angle[1];
  float yaw = mpu6050.angle[2];

  // Serial.printf("pitch: %.3f , roll: %.3f , yaw: %.3f" , pitch , roll , yaw);
  // Serial.print(pitch);
  // Serial.print(" , ");
  // Serial.print(roll);
  // Serial.print(" , ");
  // Serial.print(yaw);
  // Serial.println(" ");
}
#include <Arduino.h>
#include "imu.h"
#include "xbox.h"
#include "im948_CMD.h"

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxController xbox = XboxController("0c:35:26:e6:90:d9");

IMU imu;


void setup()
{
  Serial.begin(115200);
  imu = IMU(20,19,17);
  xbox.connectXboxController();
}

void loop()
{
    imu.update();

    Serial.print("ypr\t");
    Serial.print(imu.getYawDeg());
    Serial.print("\t");
    Serial.print(imu.getPitchDeg());
    Serial.print("\t");
    Serial.println(imu.getRollDeg());

    delay(100);
}
#include <Arduino.h>
#include "calc.h"
#include "can.h"
#include "im948_CMD.h"
#include "imu.h"
#include "msg.h"
#include "xbox.h"

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxController xbox = XboxController("ac:8e:bd:14:55:d1");

IMU imu;

void setup() {
    Serial.begin(115200);

    xbox.connectXboxController();

    //   imu = IMU(20,19,17);
    im948Init(19, 20);

    //   canInit(4, 5, TWAI_SPEED_1000KBPS , 10 , 10);
    
}

void loop() {
    Serial.println(xbox.xboxString());
    // im948ReadData();
    // delay(100);
}
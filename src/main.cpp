#include "can.h"
#include "imu.h"
#include "motor.h"
#include "my_timer.h"
#include "xbox.h"

// parameters
const int8_t rx = 20;
const int8_t tx = 19;

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxController xbox = XboxController("ac:8e:bd:14:55:d1");

IMU imu;
IMU948 imu948(rx, tx);
MyTimer timer;

void setup() {
    Serial.begin(115200);
    xbox.connectXboxController();
    imu948.init();
    canInit(4, 5, 1000, 100, 100);
    MotorInit();
}

void loop() {
    // You can set custom timeout, default is 1000
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        Serial.print("ID: ");
        Serial.print(rxFrame.identifier, HEX);
        Serial.print(" Data: ");
        for (uint8_t i = 0; i < rxFrame.data_length_code; i++) {
            Serial.print(rxFrame.data[i], HEX);
            Serial.print(" ");
        }
    }
    delay(10);
}
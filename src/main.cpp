#include "can.h"
#include "im948_CMD.h"
#include "imu.h"
#include "xbox.h"
#include "motor.h"
#include "my_timer.h"

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxController xbox = XboxController("ac:8e:bd:14:55:d1");

IMU imu;
MyTimer timer;
bool first_flag = true;


void setup() {
    Serial.begin(115200);

    // xbox.connectXboxController();

    //   imu = IMU(20,19,17);
    // im948Init(19, 20);

    canInit(5, 4, 1000, 100, 100);
}

void loop() {
    // Serial.println(xbox.xboxString());
    // im948ReadData();
    // delay(100);

    if(timer.getTimeMilliSecond() > 1000){
        if (first_flag) {
            timer.reset();
            first_flag = false;
            sendRunMotorMsg(Motor_MG5010_LF_ID);
        }else{
            // sendMotorSpeedControl(Motor_MG5010_LF_ID, 1000);
            // sendMotorReadStatus1Msg(Motor_MG5010_LF_ID);
            sendMotorTorqueControl(Motor_MG5010_LF_ID, 50);
            timer.reset();
        }
    }

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
    delay(100);
}
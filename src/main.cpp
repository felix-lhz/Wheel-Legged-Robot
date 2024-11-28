#include "can.h"
#include "im948_CMD.h"
#include "imu.h"
#include "motor.h"
#include "my_timer.h"
#include "xbox.h"

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxController xbox = XboxController("ac:8e:bd:14:55:d1");

IMU imu;
WitImu wit_imu(11,10,0x34);
MyTimer timer;
bool first_flag = true;
uint8_t hex_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

void setup() {
    Serial.begin(115200);
    // xbox.connectXboxController();

    //   imu = IMU(20,19,17);
    // im948Init(19, 20);
    wit_imu.init();

    // canInit(5, 4, 1000, 100, 100);
}

void loop() {
    // Serial.println(xbox.xboxString());
    // im948ReadData();
    // delay(100);
    // imu.update();
    // Serial.print("Yaw: ");
    // Serial.print(imu.getYawDeg());
    // Serial.print(" Pitch: ");
    // Serial.print(imu.getPitchDeg());
    // Serial.print(" Roll: ");
    // Serial.println(imu.getRollDeg());
    wit_imu.update();
    Serial.print(wit_imu.getRoll());
    Serial.print(" ");
    Serial.print(wit_imu.getPitch());
    Serial.print(" ");
    Serial.println(wit_imu.getYaw());

    // Serial.write(hex_data, sizeof(hex_data));
    // im948_serial.write(hex_data, sizeof(hex_data));

    // if(timer.getTimeMilliSecond() > 1000){
    //     if (first_flag) {
    //         timer.reset();
    //         first_flag = false;
    //         sendRunMotorMsg(Motor_MG5010_LF_ID);
    //     }else{
    //         // sendMotorSpeedControl(Motor_MG5010_LF_ID, 1000);
    //         // sendMotorReadStatus1Msg(Motor_MG5010_LF_ID);
    //         sendMotorTorqueControl(Motor_MG5010_LF_ID, 50);
    //         timer.reset();
    //     }
    // }

    // // You can set custom timeout, default is 1000
    // if (ESP32Can.readFrame(rxFrame, 1000)) {
    //     Serial.print("ID: ");
    //     Serial.print(rxFrame.identifier, HEX);
    //     Serial.print(" Data: ");
    //     for (uint8_t i = 0; i < rxFrame.data_length_code; i++) {
    //         Serial.print(rxFrame.data[i], HEX);
    //         Serial.print(" ");
    //     }
    // }
    delay(100);
}
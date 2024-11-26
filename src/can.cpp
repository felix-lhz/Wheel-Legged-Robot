#include "can.h"
#include "msg.h"

CanFrame rxFrame{0};
// CanFrame txFrame{0};

/**
 * @brief 初始化 CAN 总线
 * @param txPIN CAN 发送引脚
 * @param rxPIN CAN 接收引脚
 * @param speed CAN 速率
 * @param txQueue CAN 发送队列大小
 * @param rxQueue CAN 接收队列大小
 */
void canInit(uint8_t txPIN, uint8_t rxPIN, uint16_t speed, uint16_t txQueue,
             uint16_t rxQueue) {
    ESP32Can.setPins(txPIN, rxPIN);
    // ESP32Can.setSpeed(speed);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(speed));
    ESP32Can.setTxQueueSize(txQueue);
    ESP32Can.setRxQueueSize(rxQueue);
    while (!ESP32Can.begin()) {
        delay(100);
    }
    Serial.println("CAN init success");
}

void sendRunMotorMsg(uint32_t ID) {
    CanFrame txFrame{0};
    txFrame.identifier = ID; // Default OBD2 address;
    txFrame.extd = 0;
    txFrame.data_length_code = 8;
    for(uint8_t i = 0; i < 8; i++) {
        txFrame.data[i] = runMotorMSG[i];
    }
    // Accepts both pointers and references
    while(!ESP32Can.writeFrame(txFrame)); // timeout defaults to 1 ms
    Serial.println("Run motor msg sent");
}

void sendMotorTorqueControl(uint32_t ID, int16_t _iqControl) {
    CanFrame txFrame{0};
    txFrame.identifier = ID; // Default OBD2 address;
    txFrame.extd = 0;
    txFrame.data_length_code = 8;
    uint8_t *data = MotorTorqueClosedControl(_iqControl);
    for(uint8_t i = 0; i < 8; i++) {
        txFrame.data[i] = data[i];
    }
    // Accepts both pointers and references
    while(!ESP32Can.writeFrame(txFrame)); // timeout defaults to 1 ms
    Serial.println("Torque control msg sent");
}

void sendMotorSpeedControl(uint32_t ID, int32_t _speedControl) {
    CanFrame txFrame{0};
    txFrame.identifier = ID; // Default OBD2 address;
    txFrame.extd = 0;
    txFrame.data_length_code = 8;
    uint8_t *data = MotorSpeedClosedControl(_speedControl);
    for(uint8_t i = 0; i < 8; i++) {
        txFrame.data[i] = data[i];
    }
    // Accepts both pointers and references
    while(!ESP32Can.writeFrame(txFrame)); // timeout defaults to 1 ms
    Serial.println("Speed control msg sent");
}

void sendCloseMotorMsg(uint32_t ID) {
    CanFrame txFrame{0};
    txFrame.identifier = ID; // Default OBD2 address;
    txFrame.extd = 0;
    txFrame.data_length_code = 8;
    for(uint8_t i = 0; i < 8; i++) {
        txFrame.data[i] = closeMotorMsg[i];
    }
    // Accepts both pointers and references
    while(!ESP32Can.writeFrame(txFrame)); // timeout defaults to 1 ms
    Serial.println("Close motor msg sent");
}

void sendMotorReadStatus1Msg(uint32_t ID) {
    CanFrame txFrame{0};
    txFrame.identifier = ID; // Default OBD2 address;
    txFrame.extd = 0;
    txFrame.data_length_code = 8;
    for(uint8_t i = 0; i < 8; i++) {
        txFrame.data[i] = MotorReadStatus1Msg[i];
    }
    // Accepts both pointers and references
    while(!ESP32Can.writeFrame(txFrame)); // timeout defaults to 1 ms
    Serial.println("Read status 1 msg sent");
}
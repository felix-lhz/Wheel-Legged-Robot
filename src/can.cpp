#include "can.h"

CanFrame rxFrame{0};

/**
 * @brief 初始化 CAN 总线
 * @param txPIN CAN 发送引脚
 * @param rxPIN CAN 接收引脚
 * @param speed CAN 速率
 * @param txQueue CAN 发送队列大小
 * @param rxQueue CAN 接收队列大小
 */
void canInit(uint8_t txPIN, uint8_t rxPIN, TwaiSpeed speed, uint16_t txQueue,
             uint16_t rxQueue) {
    ESP32Can.setPins(txPIN, rxPIN);
    ESP32Can.setSpeed(speed);
    ESP32Can.setTxQueueSize(txQueue);
    ESP32Can.setRxQueueSize(rxQueue);
    while (!ESP32Can.begin()) {
        delay(100);
    }
}
#include "can.h"

CanFrame rxFrame {0};

void canInit(uint8_t txPIN, uint8_t rxPIN, TwaiSpeed speed, uint16_t txQueue,
             uint16_t rxQueue) {
    ESP32Can.setPins(txPIN, rxPIN);
    ESP32Can.setSpeed(speed);
    ESP32Can.setTxQueueSize(txQueue);
    ESP32Can.setRxQueueSize(rxQueue);
    while(!ESP32Can.begin()) {
        delay(100);
    }
}
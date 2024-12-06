#ifndef CAN_H
#define CAN_H

#include <ESP32-TWAI-CAN.hpp>

extern CanFrame rxFrame;
// extern CanFrame txFrame;

void canInit(uint8_t txPIN, uint8_t rxPIN,
             uint16_t speed = 1000, uint16_t txQueue = 10,
             uint16_t rxQueue = 10);
#endif
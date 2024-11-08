#ifndef CAN_H
#define CAN_H

#include <ESP32-TWAI-CAN.hpp>

extern CanFrame rxFrame;

void canInit(uint8_t txPIN, uint8_t rxPIN,
             TwaiSpeed speed = TWAI_SPEED_1000KBPS, uint16_t txQueue = 10,
             uint16_t rxQueue = 10);
#endif
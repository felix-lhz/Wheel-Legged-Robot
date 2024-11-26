#ifndef CAN_H
#define CAN_H

#include <ESP32-TWAI-CAN.hpp>

extern CanFrame rxFrame;
// extern CanFrame txFrame;

void canInit(uint8_t txPIN, uint8_t rxPIN,
             uint16_t speed = 1000, uint16_t txQueue = 10,
             uint16_t rxQueue = 10);

void sendRunMotorMsg(uint32_t ID);
void sendMotorTorqueControl(uint32_t ID, int16_t _iqControl);
void sendMotorSpeedControl(uint32_t ID, int32_t _speedControl);
void sendCloseMotorMsg(uint32_t ID);

void sendMotorReadStatus1Msg(uint32_t ID);
#endif
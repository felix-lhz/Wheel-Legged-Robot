#include "msg.h"

const uint32_t Motor_MF9025_L = 0x140 + 0x01;
const uint32_t Motor_MF9025_R = 0x140 + 0x02;
const uint32_t Motor_MG5010_LF = 0x140 + 0x03;
const uint32_t Motor_MG5010_RF = 0x140 + 0x04;
const uint32_t Motor_MG5010_LB = 0x140 + 0x05;
const uint32_t Motor_MG5010_RB = 0x140 + 0x06;

const uint8_t closeMotorMsg[8] = {0x80, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00}; // 电机关闭命令
const uint8_t closeMotorFeedback[8] = {0x80, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00}; // 电机关闭命令回复
const uint8_t runMotorMSG[8] = {0x88, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00}; // 电机运行命令
const uint8_t runMotorFeedback[8] = {0x88, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00}; // 电机运行命令回复
const uint8_t stopMotorMsg[8] = {0x81, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00}; // 电机停止命令
const uint8_t stopMotorFeedback[8] = {0x81, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00}; // 电机停止命令回复

const uint8_t MSMotorPowerOpenControlCommandByte = 0xA0; // 开环控制命令
const uint8_t MotorTorqueClosedControlCommandByte = 0xA1; // 转矩闭环控制命令
const uint8_t MotorSpeedClosedControlCommandByte = 0xA2; // 速度闭环控制命令
const uint8_t MotorMultiLoopsAngleClosedControlCommandByte1 =
    0xA3; // 多圈位置闭环控制命令-1
const uint8_t MotorMultiLoopsAngleClosedControlCommandByte2 =
    0xA4; // 多圈位置闭环控制命令-2
const uint8_t MotorSingleLoopAngleClosedControlCommandByte1 =
    0xA5; // 单圈位置闭环控制命令-1
const uint8_t MotorSingleLoopAngleClosedControlCommandByte2 =
    0xA6; // 单圈位置闭环控制命令-2

/**
 * @brief 开环控制命令（该命令仅在 MS 电机上实现，其他电机无效）
 * @param _powerControl 电机控制值，为 int16_t 类型，数值范围-850~
850，（电机电流和扭矩因电机而异）
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令中的控制值 powerControl 不受上位机中的 Max Power 值限制。
 */
uint8_t *MSMotorPowerOpenControl(int16_t _powerControl) {
    uint8_t *data = new uint8_t[8];
    _powerControl = constrain(_powerControl, -850, 850);
    data[0] = MSMotorPowerOpenControlCommandByte; // 命令字节
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&_powerControl);       // 开环控制值低字节
    data[5] = *((uint8_t *)(&_powerControl) + 1); // 开环控制值高字节
    data[6] = 0x00;
    data[7] = 0x00;
    return data;
}

/**
 * @brief 电机数据解析(开环控制MS)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MSMotorPowerOpenControlFeedback(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MSMotorPowerOpenControlCommandByte) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.power = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 转矩闭环控制命令（该命令仅在 MF、MH、MG
 * 电机上实现），主机发送该命令以控制电机的转矩电流输出
 * @param _iqControl 电机的转矩电流输出控制值，为 int16_t
 * 类型，数值范围-2048~2048，对应 MF 电机实际转矩电流范围 - 16.5A ~ 16.5A，对应
 * MG 电机实际转矩电流范围-33A~33A， 母线电流和电机的实际扭矩因不同电机而异。
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令中的控制值 iqControl 不受上位机中的 Max Power 值限制。
 */
uint8_t *MotorTorqueClosedControl(int16_t _iqControl) {
    uint8_t *data = new uint8_t[8];
    _iqControl = constrain(_iqControl, -2048, 2048);
    data[0] = MotorTorqueClosedControlCommandByte; // 命令字节
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&_iqControl);       // 开环控制值低字节
    data[5] = *((uint8_t *)(&_iqControl) + 1); // 开环控制值高字节
    data[6] = 0x00;
    data[7] = 0x00;
    return data;
}

/**
 * @brief 电机数据解析(转矩闭环控制,MF、MH、MG 电机)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorTorqueClosedControlFeedback(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MotorTorqueClosedControlCommandByte) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 速度闭环控制命令，主机发送该命令以控制电机的转速
 * @param _speedControl 电机的转速控制值，为 int32_t,对应实际转速为
0.01dps/LSB
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令下电机的 speedControl 由上位机中的 Max Speed 值限制。
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 */
uint8_t *MotorSpeedClosedControl(int32_t _speedControl) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorSpeedClosedControlCommandByte; // 命令字节
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&_speedControl);       // 速度控制值低字节
    data[5] = *((uint8_t *)(&_speedControl) + 1); // 速度控制值
    data[6] = *((uint8_t *)(&_speedControl) + 2); // 速度控制值
    data[7] = *((uint8_t *)(&_speedControl) + 3); // 速度控制值高字节
    return data;
}

/**
 * @brief 电机数据解析(速度闭环控制,MF、MH、MG 电机)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorSpeedClosedControlFeedback(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MotorSpeedClosedControlCommandByte) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 多圈位置闭环控制命令-1
 * @param _angleControl 电机的位置控制值，为 int32_t,对应实际位置为
 * 0.01dps/LSB,即 36000 代表 360°
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令下的控制值 angleControl 受上位机中的 Max Angle 值限制。
 * @note 该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 * */
uint8_t *MotorMultiLoopsAngleClosedControl1(int32_t _angleControl) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorMultiLoopsAngleClosedControlCommandByte1; // 命令字节
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&_angleControl);       // 位置控制值低字节
    data[5] = *((uint8_t *)(&_angleControl) + 1); // 位置控制值
    data[6] = *((uint8_t *)(&_angleControl) + 2); // 位置控制值
    data[7] = *((uint8_t *)(&_angleControl) + 3); // 位置控制值高字节
    return data;
}

/**
 * @brief 电机数据解析(多圈位置闭环控制-1)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorMultiLoopsAngleClosedControlFeedback1(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MotorMultiLoopsAngleClosedControlCommandByte1) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 多圈位置闭环控制命令-2
 * @param _angleControl 电机的位置控制值，为 int32_t,对应实际位置为
 * 0.01dps/LSB,即 36000 代表 360°
 * @param _maxSpeed 电机的最大速度，为 uint16_t,对应实际速度为 1 dps/LSB
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令下的控制值 angleControl 受上位机中的 Max Angle 值限制。
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
 * Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 */
uint8_t *MotorMultiLoopsAngleClosedControl2(int32_t _angleControl,
                                            uint16_t _maxSpeed) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorMultiLoopsAngleClosedControlCommandByte2; // 命令字节
    data[1] = 0x00;
    data[2] = *(uint8_t *)(&_maxSpeed);           // 最大速度低字节
    data[3] = *((uint8_t *)(&_maxSpeed) + 1);     // 最大速度高字节
    data[4] = *(uint8_t *)(&_angleControl);       // 位置控制值低字节
    data[5] = *((uint8_t *)(&_angleControl) + 1); // 位置控制值
    data[6] = *((uint8_t *)(&_angleControl) + 2); // 位置控制值
    data[7] = *((uint8_t *)(&_angleControl) + 3); // 位置控制值高字节
    return data;
}

/**
 * @brief 电机数据解析(多圈位置闭环控制-2)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorMultiLoopsAngleClosedControlFeedback2(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MotorMultiLoopsAngleClosedControlCommandByte2) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 单圈位置闭环控制命令-1
 * @param _angleControl 电机的位置控制值，为 int32_t,对应实际位置为
 * 0.01dps/LSB,即 36000 代表 360°
 * @param _spinDirection 电机的旋转方向，为 bool 类型，true 为正转，false 为反转
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
 * Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 */
uint8_t *MotorSingleLoopAngleClosedControl1(uint32_t _angleControl,
                                            bool _spinDirection) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorSingleLoopAngleClosedControlCommandByte1; // 命令字节
    data[1] = _spinDirection ? 0x00 : 0x01;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&_angleControl);       // 位置控制值低字节
    data[5] = *((uint8_t *)(&_angleControl) + 1); // 位置控制值
    data[6] = *((uint8_t *)(&_angleControl) + 2); // 位置控制值
    data[7] = *((uint8_t *)(&_angleControl) + 3); // 位置控制值高字节
    return data;
}

/**
 * @brief 电机数据解析(单圈位置闭环控制-1)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorSingleLoopAngleClosedControlFeedback1(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MotorSingleLoopAngleClosedControlCommandByte1) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 单圈位置闭环控制命令-2
 * @param _angleControl 电机的位置控制值，为 int32_t,对应实际位置为
 * 0.01dps/LSB,即 36000 代表 360°
 * @param _maxSpeed 电机的最大速度，为 uint16_t,对应实际速度为 1 dps/LSB
 * @param _spinDirection 电机的旋转方向，为 bool 类型，true 为正转，false 为反转
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
 * Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 */
uint8_t *MotorSingleLoopAngleClosedControl2(uint32_t _angleControl,
                                            uint16_t _maxSpeed,
                                            bool _spinDirection) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorSingleLoopAngleClosedControlCommandByte2; // 命令字节
    data[1] = _spinDirection ? 0x00 : 0x01;
    data[2] = *(uint8_t *)(&_maxSpeed);           // 最大速度低字节
    data[3] = *((uint8_t *)(&_maxSpeed) + 1);     // 最大速度高字节
    data[4] = *(uint8_t *)(&_angleControl);       // 位置控制值低字节
    data[5] = *((uint8_t *)(&_angleControl) + 1); // 位置控制值
    data[6] = *((uint8_t *)(&_angleControl) + 2); // 位置控制值
    data[7] = *((uint8_t *)(&_angleControl) + 3); // 位置控制值高字节
    return data;
}

/**
 * @brief 电机数据解析(单圈位置闭环控制-2)
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorSingleLoopAngleClosedControlFeedback2(const CanFrame frame) {
    MotorData motor_data;
    if (frame.data[0] == MotorSingleLoopAngleClosedControlCommandByte2) {
        motor_data.is_valid = true;
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}
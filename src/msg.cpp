#include "msg.h"

const uint32_t Motor_MF9025_L = 0x140 + 0x01;
const uint32_t Motor_MF9025_R = 0x140 + 0x02;
const uint32_t Motor_MG5010_LF = 0x140 + 0x03;
const uint32_t Motor_MG5010_RF = 0x140 + 0x04;
const uint32_t Motor_MG5010_LB = 0x140 + 0x05;
const uint32_t Motor_MG5010_RB = 0x140 + 0x06;

const uint8_t closeMotorMsg[8] = {0x80, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00}; // 电机关闭命令
const uint8_t closeMotorCommandByte = 0x80;                // 电机关闭命令
const uint8_t runMotorMSG[8] = {0x88, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00}; // 电机运行命令
const uint8_t runMotorCommandByte = 0x88;                // 电机运行命令
const uint8_t stopMotorMsg[8] = {0x81, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00}; // 电机停止命令
const uint8_t stopMotorCommandByte = 0x81;                // 电机停止命令

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
const uint8_t MotorIncrementalAngleClosedControlCommandByte1 =
    0xA7; // 增量位置闭环控制命令-1
const uint8_t MotorIncrementalAngleClosedControlCommandByte2 =
    0xA8; // 增量位置闭环控制命令-2

const uint8_t MotorReadPIDParamMsg[8] = {0x30, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00}; // 读取 PID 参数命令
const uint8_t MotorReadPIDParamCommandByte = 0x30; // 读取 PID 参数命令字节
const uint8_t MotorWritePIDParamToRAMCommandByte =
    0x31; // 写入 PID 参数到 RAM 命令字节
const uint8_t MotorWritePIDParamToROMCommandByte =
    0x32; // 写入 PID 参数到 ROM 命令字节

/**
 * @brief 电机基本命令字节判断
 * @param _commandByte 命令字节
 * @return bool 是否为基本电机命令字节
 * @note 电机基本命令字节包括关闭、运行、停止命令
 * */
bool BasicMotorCommandByteJudge(const uint8_t _commandByte) {
    switch (_commandByte) {
    case closeMotorCommandByte:
        break;
    case runMotorCommandByte:
        break;
    case stopMotorCommandByte:
        break;
    default:
        return false;
    }
    return true;
}

/**
 * @brief 电机控制命令字节判断
 * @param _commandByte 命令字节
 * @return bool 是否为电机控制命令字节
 * @note
 * 电机控制命令字节包括开环、转矩闭环、速度闭环、多圈位置闭环、单圈位置闭环、增量位置闭环命令
 * */
bool MotorControlCommandByteJudge(const uint8_t _commandByte) {
    if (_commandByte >= 0xA1 && _commandByte <= 0xA8) {
        return true;
    }
    return false;
}

/**
 * @brief 电机PID参数命令字节判断
 * @param _commandByte 命令字节
 * @return bool 是否为电机PID参数命令字节
 * @note
 * 电机PID参数命令字节包括读取PID参数、写入PID参数到RAM、写入PID参数到ROM命令
 * */
bool MotorPIDParamCommandByteJudge(const uint8_t _commandByte) {
    if (_commandByte >= 0x30 && _commandByte <= 0x32) {
        return true;
    }
    return false;
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
 * @brief 增量位置闭环控制命令-1
 * @param _angleIncrement 电机的位置增量控制值，为 int32_t,对应实际位置为
 * 0.01dps/LSB,即 36000 代表 360°,正值为正转，负值为反转
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该命令下电机的最大速度由上位机中的 Max Speed 值限制。
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
 * Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 */
uint8_t *MotorIncrementalAngleClosedControl1(int32_t _angleIncrement) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorIncrementalAngleClosedControlCommandByte1; // 命令字节
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = *(uint8_t *)(&_angleIncrement);       // 位置控制值低字节
    data[5] = *((uint8_t *)(&_angleIncrement) + 1); // 位置控制值
    data[6] = *((uint8_t *)(&_angleIncrement) + 2); // 位置控制值
    data[7] = *((uint8_t *)(&_angleIncrement) + 3); // 位置控制值高字节
    return data;
}

/**
 * @brief 增量位置闭环控制命令-2
 * @param _angleIncrement 电机的位置增量控制值，为 int32_t,对应实际位置为
 * 0.01dps/LSB,即 36000 代表 360°,正值为正转，负值为反转
 * @param _maxSpeed 电机的最大速度，为 uint16_t,对应实际速度为 1 dps/LSB
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 * @note 该控制模式下，电机的最大加速度由上位机中的 Max Acceleration 值限制。
 * @note 该控制模式下，MF、MH、MG 电机的最大转矩电流由上位机中的 Max Torque
 * Current 值限制；MS 电机的最大功率由上位机中的 Max Power 值限制。
 */
uint8_t *MotorIncrementalAngleClosedControl2(int32_t _angleIncrement,
                                             uint16_t _maxSpeed) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorIncrementalAngleClosedControlCommandByte2; // 命令字节
    data[1] = 0x00;
    data[2] = *(uint8_t *)(&_maxSpeed);             // 最大速度低字节
    data[3] = *((uint8_t *)(&_maxSpeed) + 1);       // 最大速度高字节
    data[4] = *(uint8_t *)(&_angleIncrement);       // 位置控制值低字节
    data[5] = *((uint8_t *)(&_angleIncrement) + 1); // 位置控制值
    data[6] = *((uint8_t *)(&_angleIncrement) + 2); // 位置控制值
    data[7] = *((uint8_t *)(&_angleIncrement) + 3); // 位置控制值高字节
    return data;
}

/**
 * @brief 电机数据解析
 * @param frame 电机反馈数据帧
 * @return MotorData 电机数据结构体
 */
MotorData MotorControlFeedback(const CanFrame frame) {
    MotorData motor_data;
    motor_data.is_valid = MotorControlCommandByteJudge(frame.data[0]);
    if (motor_data.is_valid) {
        motor_data.motor_id = frame.identifier;
        motor_data.temperature = frame.data[1];
        motor_data.iq = (int16_t)(frame.data[3] << 8 | frame.data[2]);
        motor_data.speed = (int16_t)(frame.data[5] << 8 | frame.data[4]);
        motor_data.angle = (uint16_t)(frame.data[7] << 8 | frame.data[6]);
    }
    return motor_data;
}

/**
 * @brief 主机发送该命令写入 PID 参数到 RAM 中，断电后写入参数失效
 * @param _anglePID_P 角度环P
 * @param _anglePID_I 角度环I
 * @param _speedPID_P 速度环P
 * @param _speedPID_I 速度环I
 * @param _iqPID_P 电流环P
 * @param _iqPID_I 电流环I
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 */
uint8_t *MotorWritePIDParamToRAM(uint8_t _anglePID_P, uint8_t _anglePID_I,
                                 uint8_t _speedPID_P, uint8_t _speedPID_I,
                                 uint8_t _iqPID_P, uint8_t _iqPID_I) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorWritePIDParamToRAMCommandByte; // 命令字节
    data[1] = 0x00;                               // NULL
    data[2] = _anglePID_P;                        // 角度环P
    data[3] = _anglePID_I;                        // 角度环I
    data[4] = _speedPID_P;                        // 速度环P
    data[5] = _speedPID_I;                        // 速度环I
    data[6] = _iqPID_P;                           // 电流环P
    data[7] = _iqPID_I;                           // 电流环I
    return data;
}

/**
 * @brief 主机发送该命令写入 PID 参数到 ROM 中，断电后写入参数不失效
 * @param _anglePID_P 角度环P
 * @param _anglePID_I 角度环I
 * @param _speedPID_P 速度环P
 * @param _speedPID_I 速度环I
 * @param _iqPID_P 电流环P
 * @param _iqPID_I 电流环I
 * @return uint8_t* 返回一个长度为 8 的数组，包含 CAN 命令
 */
uint8_t *MotorWritePIDParamToROM(uint8_t _anglePID_P, uint8_t _anglePID_I,
                                 uint8_t _speedPID_P, uint8_t _speedPID_I,
                                 uint8_t _iqPID_P, uint8_t _iqPID_I) {
    uint8_t *data = new uint8_t[8];
    data[0] = MotorWritePIDParamToROMCommandByte; // 命令字节
    data[1] = 0x00;                               // NULL
    data[2] = _anglePID_P;                        // 角度环P
    data[3] = _anglePID_I;                        // 角度环I
    data[4] = _speedPID_P;                        // 速度环P
    data[5] = _speedPID_I;                        // 速度环I
    data[6] = _iqPID_P;                           // 电流环P
    data[7] = _iqPID_I;                           // 电流环I
    return data;
}

/**
 * @brief 电机PID参数反馈解析
 * @param frame 电机PID参数反馈数据帧
 * @return MotorPIDParam 电机PID参数结构体
 */
MotorPIDParam MotorReadPIDParamFeedback(const CanFrame frame) {
    MotorPIDParam motor_pid_param;
    motor_pid_param.is_valid = MotorPIDParamCommandByteJudge(frame.data[0]);
    if (motor_pid_param.is_valid) {
        motor_pid_param.motor_id = frame.identifier;
        motor_pid_param.anglePID_P = frame.data[2];
        motor_pid_param.anglePID_I = frame.data[3];
        motor_pid_param.speedPID_P = frame.data[4];
        motor_pid_param.speedPID_I = frame.data[5];
        motor_pid_param.iqPID_P = frame.data[6];
        motor_pid_param.iqPID_I = frame.data[7];
    }
    return motor_pid_param;
}

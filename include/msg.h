#ifndef MSG_H
#define MSG_H

#include "can.h"

// define Motor Controller CAN IDs
extern const uint32_t Motor_MF9025_L;
extern const uint32_t Motor_MF9025_R;
extern const uint32_t Motor_MG5010_LF;
extern const uint32_t Motor_MG5010_RF;
extern const uint32_t Motor_MG5010_LB;
extern const uint32_t Motor_MG5010_RB;

/**
 * @brief 电机关闭命令
 * @note 将电机从开启状态（上电后默认状态）切换到关闭状态
 * @note 清除电机转动圈数及之前接收的控制指令，LED 由常亮转为慢闪
 * @note 电机仍然可以回复控制命令，但不会执行动作
 * @return 和主机发送相同
 */
extern const uint8_t closeMotorMsg[8];
extern const uint8_t closeMotorCommandByte;

/**
 * @brief 电机运行命令
 * @note 将电机从关闭状态切换到开启状态，LED 由慢闪转为常亮
 * @note 此时再发送控制指令即可控制电机动作
 * @return 和主机发送相同
 */
extern const uint8_t runMotorMSG[8];
extern const uint8_t runMotorCommandByte;

/**
 * @brief 电机停止命令
 * @note 将电机从开启状态切换到关闭状态
 * @note 但不清除电机转动圈数及之前接收的控制指令
 * @note 电机仍然可以回复控制命令且会执行动作
 * @return 和主机发送相同
 */
extern const uint8_t stopMotorMsg[8];
extern const uint8_t stopMotorCommandByte;

// 开环控制命令字节（该命令仅在 MS 电机上实现，其他电机无效）
extern const uint8_t MSMotorPowerOpenControlCommandByte;

// 转矩闭环控制命令字节（该命令仅在 MF、MH、MG 电机上实现）
extern const uint8_t MotorTorqueClosedControlCommandByte;
uint8_t *MotorTorqueClosedControl(int16_t _iqControl);

// 速度闭环控制命令字节
extern const uint8_t MotorSpeedClosedControlCommandByte;
uint8_t *MotorSpeedClosedControl(int32_t _speedControl);

// 多圈位置闭环控制命令字节-1
extern const uint8_t MotorMultiLoopsAngleClosedControlCommandByte1;
uint8_t *MotorMultiLoopsAngleClosedControl1(int32_t _angleControl);

// 多圈位置闭环控制命令字节-2
extern const uint8_t MotorMultiLoopsAngleClosedControlCommandByte2;
uint8_t *MotorMultiLoopsAngleClosedControl2(int32_t _angleControl,
                                            uint16_t _maxSpeed);

// 单圈位置闭环控制命令字节-1
extern const uint8_t MotorSingleLoopAngleClosedControlCommandByte1;
uint8_t *MotorSingleLoopAngleClosedControl1(uint32_t _angleControl,
                                            bool _spinDirection = true);

// 单圈位置闭环控制命令字节-2
extern const uint8_t MotorSingleLoopAngleClosedControlCommandByte2;
uint8_t *MotorSingleLoopAngleClosedControl2(uint32_t _angleControl,
                                            uint16_t _maxSpeed,
                                            bool _spinDirection = true);

// 增量位置闭环控制命令字节-1
extern const uint8_t MotorIncrementalAngleClosedControlCommandByte1;
uint8_t *MotorIncrementalAngleClosedControl1(int32_t _angleIncrement);

// 增量位置闭环控制命令字节-2
extern const uint8_t MotorIncrementalAngleClosedControlCommandByte2;
uint8_t *MotorIncrementalAngleClosedControl2(int32_t _angleIncrement,
                                             uint16_t _maxSpeed);

// 读取PID参数命令
extern const uint8_t MotorReadPIDParamMsg[8];
extern const uint8_t MotorReadPIDParamCommandByte; // 读取 PID 参数命令字节

// 写入PID参数到RAM命令字节
extern const uint8_t MotorWritePIDParamToRAMCommandByte;
uint8_t *MotorWritePIDParamToRAM(uint8_t _anglePID_P, uint8_t _anglePID_I,
                                 uint8_t _speedPID_P, uint8_t _speedPID_I,
                                 uint8_t _iqPID_P, uint8_t _iqPID_I);

// 写入PID参数到ROM命令字节
extern const uint8_t MotorWritePIDParamToROMCommandByte;
uint8_t *MotorWritePIDParamToROM(uint8_t _anglePID_P, uint8_t _anglePID_I,
                                 uint8_t _speedPID_P, uint8_t _speedPID_I,
                                 uint8_t _iqPID_P, uint8_t _iqPID_I);

// 读取加速度命令
extern const uint8_t MotorReadAccelerationMsg[8];
extern const uint8_t MotorReadAccelerationCommandByte;

// 写入加速度到RAM命令指令
extern const uint8_t MotorWriteAccelerationToRAMCommandByte;
uint8_t *MotorWriteAccelerationToRAM(int32_t _acceleration);

// 读取编码器数据命令
extern const uint8_t MotorReadEncoderMsg[8];
extern const uint8_t MotorReadEncoderCommandByte; // 读取编码器数据命令字节

// 写入编码器值到ROM作为电机零点命令字节
extern const uint8_t MotorWriteEncoderToROMCommandByte;
uint8_t *MotorWriteEncoderToROM(uint16_t _encoderOffset);

// 写入当前位置到ROM作为电机零点命令
extern const uint8_t MotorWriteCurrentPositionToROMMsg[8];
extern const uint8_t MotorWriteCurrentPositionToROMCommandByte;

// 读取多圈角度命令
extern const uint8_t MotorReadMultiLoopsAngleMsg[8];
extern const uint8_t MotorReadMultiLoopsAngleCommandByte;

// 读取单圈角度命令
extern const uint8_t MotorReadSingleLoopAngleMsg[8];
extern const uint8_t MotorReadSingleLoopAngleCommandByte;

// 清除电机角度命令【暂未实现】，该命令清除电机的多圈和单圈角度数据，
// 并将当前位置设为电机的零点，断电后失效，
// 注意：该命令会同时清除所有位置环的控制命令数据
extern const uint8_t MotorClearAngleMsg[8];
extern const uint8_t MotorClearAngleCommandByte;

// 读取电机状态-1和错误标志命令：该命令读取当前电机的温度、电压和错误状态标志
extern const uint8_t MotorReadStatus1Msg[8];
extern const uint8_t MotorReadStatus1CommandByte;

//清楚电机错误标志命令：该命令清除电机的错误标志
extern const uint8_t MotorClearErrorFlagMsg[8];
extern const uint8_t MotorClearErrorFlagCommandByte;

// 读取电机状态-2命令：该命令读取当前电机的电压、转速、编码器位置
extern const uint8_t MotorReadStatus2Msg[8];
extern const uint8_t MotorReadStatus2CommandByte;

// 读取电机状态-3命令：该命令读取当前电机的温度和相电流数据
extern const uint8_t MotorReadStatus3Msg[8];
extern const uint8_t MotorReadStatus3CommandByte;

#endif // MSG_H
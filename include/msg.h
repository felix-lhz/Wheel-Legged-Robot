#ifndef MSG_H
#define MSG_H

#include "can.h"

/*
 * @brief 电机数据结构体
 * @param motor_id 电机 ID
 * @param temperature 电机温度：1℃/LSB
 * @param iq 电机电流：-2048~2048 , MF 电机实际转矩电流范围 - 16.5A ~ 16.5A，MG
 * 电机实际转矩电流范围-33A~33A
 * @param power 电机输出功率：-850~850
 * @param speed 电机转速:1dps/LSB
 * @param angle 电机位置：0~32767(15bit编码器) ， 0~65535(16bit编码器)
 */
struct MotorData {
    bool is_valid = false;
    uint32_t motor_id = 0;
    int8_t temperature = 0;
    int16_t iq = 0;
    int16_t speed = 0;
    uint16_t angle = 0;
};

struct MotorPIDParam {
    bool is_valid = false;
    uint32_t motor_id = 0;
    uint8_t anglePID_P = 0;
    uint8_t anglePID_I = 0;
    uint8_t speedPID_P = 0;
    uint8_t speedPID_I = 0;
    uint8_t iqPID_P = 0;
    uint8_t iqPID_I = 0;
};

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

bool BasicMotorCommandByteJudge(const uint8_t _commandByte);
bool MotorControlCommandByteJudge(const uint8_t _commandByte);
bool MotorPIDParamCommandByteJudge(const uint8_t _commandByte);
bool MotorAccelerationCommandByteJudge(const uint8_t _commandByte);

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
uint8_t *MotorSingleLoopAngleClosedControl1(uint32_t _angleControl , bool _spinDirection = true);

// 单圈位置闭环控制命令字节-2
extern const uint8_t MotorSingleLoopAngleClosedControlCommandByte2;
uint8_t *MotorSingleLoopAngleClosedControl2(uint32_t _angleControl ,uint16_t _maxSpeed , bool _spinDirection = true);

// 增量位置闭环控制命令字节-1
extern const uint8_t MotorIncrementalAngleClosedControlCommandByte1;
uint8_t *MotorIncrementalAngleClosedControl1(int32_t _angleIncrement);

// 增量位置闭环控制命令字节-2
extern const uint8_t MotorIncrementalAngleClosedControlCommandByte2;
uint8_t *MotorIncrementalAngleClosedControl2(int32_t _angleIncrement, uint16_t _maxSpeed);

MotorData MotorControlFeedback(const CanFrame frame);

// 读取PID参数命令
extern const uint8_t MotorReadPIDParamMsg[8];
extern const uint8_t MotorReadPIDParamCommandByte; // 读取 PID 参数命令字节

//写入PID参数到RAM命令字节
extern const uint8_t MotorWritePIDParamToRAMCommandByte;
uint8_t *MotorWritePIDParamToRAM(uint8_t _anglePID_P, uint8_t _anglePID_I, uint8_t _speedPID_P, uint8_t _speedPID_I, uint8_t _iqPID_P, uint8_t _iqPID_I);

//写入PID参数到ROM命令字节
extern const uint8_t MotorWritePIDParamToROMCommandByte;
uint8_t *MotorWritePIDParamToROM(uint8_t _anglePID_P, uint8_t _anglePID_I, uint8_t _speedPID_P, uint8_t _speedPID_I, uint8_t _iqPID_P, uint8_t _iqPID_I);

MotorPIDParam MotorReadPIDParamFeedback(const CanFrame frame);

// 读取加速度命令
extern const uint8_t MotorReadAccelerationMsg[8];
extern const uint8_t MotorReadAccelerationCommandByte;

// 写入加速度到RAM命令指令
extern const uint8_t MotorWriteAccelerationToRAMCommandByte;
uint8_t *MotorWriteAccelerationToRAM(int32_t _acceleration);

int32_t MotorReadAccelerationFeedback(const CanFrame frame);

#endif // MSG_H
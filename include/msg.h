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
    int16_t power = 0;
    int16_t speed = 0;
    uint16_t angle = 0;
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
extern const uint8_t closeMotorFeedback[8];

/**
 * @brief 电机运行命令
 * @note 将电机从关闭状态切换到开启状态，LED 由慢闪转为常亮
 * @note 此时再发送控制指令即可控制电机动作
 * @return 和主机发送相同
 */
extern const uint8_t runMotorMSG[8];
extern const uint8_t runMotorFeedback[8];

/**
 * @brief 电机停止命令
 * @note 将电机从开启状态切换到关闭状态
 * @note 但不清除电机转动圈数及之前接收的控制指令
 * @note 电机仍然可以回复控制命令且会执行动作
 * @return 和主机发送相同
 */
extern const uint8_t stopMotorMsg[8];
extern const uint8_t stopMotorFeedback[8];

// 开环控制命令字节（该命令仅在 MS 电机上实现，其他电机无效）
extern const uint8_t MSMotorPowerOpenControlCommandByte; 
uint8_t *MSMotorPowerOpenControl(int16_t _powerControl);
MotorData MSMotorPowerOpenControlFeedback(const CanFrame frame);

// 转矩闭环控制命令字节（该命令仅在 MF、MH、MG 电机上实现）
extern const uint8_t MotorTorqueClosedControlCommandByte; 
uint8_t *MotorTorqueClosedControl(int16_t _iqControl);
MotorData MotorTorqueClosedControlFeedback(const CanFrame frame);

// 速度闭环控制命令字节
extern const uint8_t MotorSpeedClosedControlCommandByte;
uint8_t *MotorSpeedClosedControl(int32_t _speedControl);
MotorData MotorSpeedClosedControlFeedback(const CanFrame frame);

// 多圈位置闭环控制命令字节-1
extern const uint8_t MotorMultiLoopsAngleClosedControlCommandByte1;
uint8_t *MotorMultiLoopsAngleClosedControl1(int32_t _angleControl);
MotorData MotorMultiLoopsAngleClosedControlFeedback1(const CanFrame frame);

// 多圈位置闭环控制命令字节-2
extern const uint8_t MotorMultiLoopsAngleClosedControlCommandByte2;
uint8_t *MotorMultiLoopsAngleClosedControl2(int32_t _angleControl,
                                            uint16_t _maxSpeed);
MotorData MotorMultiLoopsAngleClosedControlFeedback2(const CanFrame frame);

// 单圈位置闭环控制命令字节-1
extern const uint8_t MotorSingleLoopAngleClosedControlCommandByte1;
uint8_t *MotorSingleLoopAngleClosedControl1(uint32_t _angleControl , bool _spinDirection = true);
MotorData MotorSingleLoopAngleClosedControlFeedback1(const CanFrame frame);

// 单圈位置闭环控制命令字节-2
extern const uint8_t MotorSingleLoopAngleClosedControlCommandByte2;
uint8_t *MotorSingleLoopAngleClosedControl2(uint32_t _angleControl ,uint16_t _maxSpeed , bool _spinDirection = true);
MotorData MotorSingleLoopAngleClosedControlFeedback2(const CanFrame frame);

#endif // MSG_H
#ifndef MOTOR_H
#define MOTOR_H

#include "msg.h"

enum MotorType { MS_Motor, MF, MG, MH };

extern const double iq_MAX;
extern const double MF_I_MAX;
extern const double MG_I_MAX;
extern const double Encoder_18bit;

// define Motor Controller CAN IDs
extern const uint32_t Motor_MF9025_L_ID;
extern const uint32_t Motor_MF9025_R_ID;
extern const uint32_t Motor_MG5010_LF_ID;
extern const uint32_t Motor_MG5010_RF_ID;
extern const uint32_t Motor_MG5010_LB_ID;
extern const uint32_t Motor_MG5010_RB_ID;

class Motor {
  private:
    MotorType _type;         // 电机类型
    uint32_t _id;            // 电机 ID
    int8_t _temperature = 0; // 电机温度 temperature，int8_t 类型，1℃/LSB
    int16_t _iq = 0;         // 电机的实际转矩电流 iq，-2048~2048
    int16_t _speed = 0;      // 电机的实际转速 speed，1 dps/LSB
    uint16_t _encoder = 0;        // MF9025和MG5010-36E均为18bit，0~65535
    uint16_t _encoder_raw = 0;    // 编码器原始位置
    uint16_t _encoder_offset = 0; // 编码器零点位置
    uint8_t _angle_kp = 0;        // 电机的角度环P
    uint8_t _angle_ki = 0;        // 电机的角度环I
    uint8_t _speed_kp = 0;        // 电机的速度环P
    uint8_t _speed_ki = 0;        // 电机的速度环I
    uint8_t _iq_kp = 0;           // 电机的电流环P
    uint8_t _iq_ki = 0;           // 电机的电流环I
    int32_t _acceleration = 0;    // 电机的加速度，1 dps/s
    int64_t _angle = 0;           // 电机的角度，0.01 dps/LSB
    uint32_t _circle_angle = 0; // 电机的单圈角度，0.01 dps/LSB 0~36000
    uint16_t _voltage = 0;      // 电机的电压，0.1V/LSB
    uint8_t _error_state =
        0; // 电机的错误状态标志,0位：0：电压正常、1：低压保护
           // ，3位：0：温度正常、1：过温保护
    int16_t _iA = 0; // 电机的A相电流，对应实际电流1A/64LSB
    int16_t _iB = 0; // 电机的B相电流，对应实际电流1A/64LSB
    int16_t _iC = 0; // 电机的C相电流，对应实际电流1A/64LSB

    double _I = 0; // 电机的实际转矩电流,MF:-16.5A~16.5A,MG:-33A~33A
    double _position = 0;         // 电机的实际角度，单位°
    double _angle_len = 0;        // 累计角度，单位°
    double _circle_angle_len = 0; // 累计单圈角度，单位°
    double _V = 0;                // 电机的实际电压，单位V
    double _IA = 0;               // 电机的A相电流，对应实际电流1A
    double _IB = 0;               // 电机的B相电流，对应实际电流1A
    double _IC = 0;               // 电机的C相电流，对应实际电流1A

  public:
    Motor(MotorType type, uint32_t id);
    ~Motor();
    void updataCommon(int8_t temperature, int16_t iq, int16_t speed,
                      uint16_t encoder);
    void updatePID(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                   uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
    void updateAcceleration(int32_t acceleration);
    void updateEncoderRead(uint16_t encoder, uint16_t encoder_raw,
                           uint16_t encoder_offset);
    void updateEncoderOffset(uint16_t encoder_offset);
    void updateMultiLoopsAngle(int64_t angle);
    void updateSingleLoopAngle(uint32_t circle_angle);
    void updateState1(int8_t temperature, uint16_t voltage,
                      uint8_t error_state);
    void updateState2(int8_t temperature, int16_t iq, int16_t speed,
                      uint16_t encoder);
    void updateState3(int8_t temperature, int16_t iA, int16_t iB, int16_t iC);

    MotorType getType() { return _type; }
    uint32_t getID() { return _id; }
    int8_t getTemperature() { return _temperature; }
    int16_t getIq() { return _iq; }
    int16_t getSpeed() { return _speed; }
    uint16_t getEncoder() { return _encoder; }
    uint16_t getEncoderRaw() { return _encoder_raw; }
    uint16_t getEncoderOffset() { return _encoder_offset; }
    uint8_t getAngleKp() { return _angle_kp; }
    uint8_t getAngleKi() { return _angle_ki; }
    uint8_t getSpeedKp() { return _speed_kp; }
    uint8_t getSpeedKi() { return _speed_ki; }
    uint8_t getIqKp() { return _iq_kp; }
    uint8_t getIqKi() { return _iq_ki; }
    int32_t getAcceleration() { return _acceleration; }
    int64_t getAngle() { return _angle; }
    uint32_t getCircleAngle() { return _circle_angle; }
    int16_t getVoltage() { return _voltage; }
    uint8_t getErrorState() { return _error_state; }
    int16_t getiA() { return _iA; }
    int16_t getiB() { return _iB; }
    int16_t getiC() { return _iC; }
    double getI() { return _I; }
    double getPosition() { return _position; }
    double getAngleLen() { return _angle_len; }
    double getCircleAngleLen() { return _circle_angle_len; }
    double getV() { return _V; }
    double getIA() { return _IA; }
    double getIB() { return _IB; }
    double getIC() { return _IC; }
};

extern Motor Motor_MF9025_L;
extern Motor Motor_MF9025_R;
extern Motor Motor_MG5010_LF;
extern Motor Motor_MG5010_RF;
extern Motor Motor_MG5010_LB;
extern Motor Motor_MG5010_RB;

Motor *getMotorByFrameID(uint32_t frame_id);

void updateMotor(const CanFrame frame);

#endif // MOTOR_H
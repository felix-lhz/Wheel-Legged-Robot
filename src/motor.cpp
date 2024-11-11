#include "motor.h"

const double iq_MAX = 2048.0;
const double MF_I_MAX = 16.5; // A
const double MG_I_MAX = 33.0; // A
const double Encoder_18bit = 65535;

const uint32_t Motor_MF9025_L_ID = 0x140 + 0x01;
const uint32_t Motor_MF9025_R_ID = 0x140 + 0x02;
const uint32_t Motor_MG5010_LF_ID = 0x140 + 0x03;
const uint32_t Motor_MG5010_RF_ID = 0x140 + 0x04;
const uint32_t Motor_MG5010_LB_ID = 0x140 + 0x05;
const uint32_t Motor_MG5010_RB_ID = 0x140 + 0x06;

Motor Motor_MF9025_L = Motor(MF, Motor_MF9025_L_ID);
Motor Motor_MF9025_R = Motor(MF, Motor_MF9025_R_ID);
Motor Motor_MG5010_LF = Motor(MG, Motor_MG5010_LF_ID);
Motor Motor_MG5010_RF = Motor(MG, Motor_MG5010_RF_ID);
Motor Motor_MG5010_LB = Motor(MG, Motor_MG5010_LB_ID);
Motor Motor_MG5010_RB = Motor(MG, Motor_MG5010_RB_ID);

Motor::Motor(MotorType type, uint32_t id) {
    _type = type;
    _id = id;
}
Motor::~Motor() {}

void Motor::updataCommon(int8_t temperature, int16_t iq, int16_t speed,
                         uint16_t encoder) {
    _temperature = temperature;
    _iq = iq;
    _speed = speed;
    _encoder = encoder;
    int8_t sign = _iq >= 0 ? 1 : -1;
    switch (_type) {
    case MF:
        _I = (_iq * MF_I_MAX) / iq_MAX;
        break;
    case MG:
        _I = (_iq * MG_I_MAX) / iq_MAX;
        break;
    }
    _position = (encoder * 360) / Encoder_18bit;
}

void Motor::updatePID(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                      uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki) {
    _angle_kp = angle_kp;
    _angle_ki = angle_ki;
    _speed_kp = speed_kp;
    _speed_ki = speed_ki;
    _iq_kp = iq_kp;
    _iq_ki = iq_ki;
}

void Motor::updateAcceleration(int32_t acceleration) {
    _acceleration = acceleration;
}

void Motor::updateEncoderRead(uint16_t encoder, uint16_t encoder_raw,
                              uint16_t encoder_offset) {
    _encoder = encoder;
    _encoder_raw = encoder_raw;
    _encoder_offset = encoder_offset;
}

void Motor::updateEncoderOffset(uint16_t encoder_offset) {
    _encoder_offset = encoder_offset;
}

void Motor::updateMultiLoopsAngle(int64_t angle) {
    _angle = angle;
    _angle_len = angle / 100.0;
}

void Motor::updateSingleLoopAngle(uint32_t circle_angle) {
    _circle_angle = circle_angle;
    _circle_angle_len = circle_angle / 100.0;
}

void Motor::updateState1(int8_t temperature, uint16_t voltage,
                         uint8_t error_state) {
    _voltage = voltage;
    _error_state = error_state;
}

void Motor::updateState2(int8_t temperature, int16_t iq, int16_t speed,
                         uint16_t encoder) {
    _temperature = temperature;
    _iq = iq;
    _speed = speed;
    _encoder = encoder;
}

void Motor::updateState3(int8_t temperature, int16_t iA, int16_t iB,
                         int16_t iC) {
    _temperature = temperature;
    _iA = iA;
    _iB = iB;
    _iC = iC;
    _IA = iA / 64.0;
    _IB = iB / 64.0;
    _IC = iC / 64.0;
}

Motor *getMotorByFrameID(uint32_t frame_id) {
    switch (frame_id) {
    case Motor_MF9025_L_ID:
        return &Motor_MF9025_L;
    case Motor_MF9025_R_ID:
        return &Motor_MF9025_R;
    case Motor_MG5010_LF_ID:
        return &Motor_MG5010_LF;
    case Motor_MG5010_RF_ID:
        return &Motor_MG5010_RF;
    case Motor_MG5010_LB_ID:
        return &Motor_MG5010_LB;
    case Motor_MG5010_RB_ID:
        return &Motor_MG5010_RB;
    default:
        return nullptr; // 如果 frame_id 不匹配，返回空指针
    }
    return nullptr; // 如果 frame_id 不匹配，返回空指针
}

void updateMotor(const CanFrame frame) {
    MsgState state = MotorCanCommandByteJudge(frame.data[0]);
    Motor *motor = getMotorByFrameID(frame.identifier);
    switch (state) {
    case Common: {
        int8_t temperature = frame.data[1];
        int16_t iq = (frame.data[3] << 8) | frame.data[2];
        int16_t speed = (frame.data[5] << 8) | frame.data[4];
        uint16_t encoder = (frame.data[7] << 8) | frame.data[6];
        motor->updataCommon(temperature, iq, speed, encoder);
        break;
    }
    case PID: {
        uint8_t angle_kp = frame.data[2];
        uint8_t angle_ki = frame.data[3];
        uint8_t speed_kp = frame.data[4];
        uint8_t speed_ki = frame.data[5];
        uint8_t iq_kp = frame.data[6];
        uint8_t iq_ki = frame.data[7];
        motor->updatePID(angle_kp, angle_ki, speed_kp, speed_ki, iq_kp, iq_ki);
        break;
    }
    case Acceleration: {
        int32_t acceleration = (frame.data[7] << 24) | (frame.data[6] << 16) |
                               (frame.data[5] << 8) | frame.data[4];
        motor->updateAcceleration(acceleration);
        break;
    }
    case EncoderRead: {
        uint16_t encoder = (frame.data[3] << 8) | frame.data[2];
        uint16_t encoder_raw = (frame.data[5] << 8) | frame.data[4];
        uint16_t encoder_offset = (frame.data[7] << 8) | frame.data[6];
        motor->updateEncoderRead(encoder, encoder_raw, encoder_offset);
        break;
    }
    case EncoderWrite: {
        uint16_t encoder_offset = (frame.data[7] << 8) | frame.data[6];
        motor->updateEncoderOffset(encoder_offset);
        break;
    }
    case MultiLoopsAngle: {
        int64_t angle = (frame.data[7] << 48) | (frame.data[6] << 40) |
                        (frame.data[5] << 32) | (frame.data[4] << 24) |
                        (frame.data[3] << 16) | (frame.data[2] << 8) |
                        frame.data[1];
        motor->updateMultiLoopsAngle(angle);
        break;
    }
    case SingleLoopAngle: {
        uint32_t circle_angle = (frame.data[7] << 24) | (frame.data[6] << 16) |
                                (frame.data[5] << 8) | frame.data[4];
        motor->updateSingleLoopAngle(circle_angle);
        break;
    }
    case State1: {
        int8_t temperature = frame.data[1];
        uint16_t voltage = (frame.data[4] << 8) | frame.data[3];
        uint8_t error_state = frame.data[7];
        motor->updateState1(temperature, voltage, error_state);
        break;
    }
    case State2: {
        int8_t temperature = frame.data[1];
        int16_t iq = (frame.data[3] << 8) | frame.data[2];
        int16_t speed = (frame.data[5] << 8) | frame.data[4];
        uint16_t encoder = (frame.data[7] << 8) | frame.data[6];
        motor->updateState2(temperature, iq, speed, encoder);
        break;
    }
    case State3: {
        int8_t temperature = frame.data[1];
        int16_t iA = (frame.data[3] << 8) | frame.data[2];
        int16_t iB = (frame.data[5] << 8) | frame.data[4];
        int16_t iC = (frame.data[7] << 8) | frame.data[6];
        motor->updateState3(temperature, iA, iB, iC);
        break;
    }
    default: {
        Serial.println("Error: Can't match the state");
        break;
    }
    }
}
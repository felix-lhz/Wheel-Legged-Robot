#include "motor.h"

const double iq_MAX = 2048.0;
const double MF_I_MAX = 16.5; // A
const double MG_I_MAX = 33.0; // A
const double Encoder_18bit = 65535;

const uint32_t Motor_MF9025_L_ID = 0x141;
const uint32_t Motor_MF9025_R_ID = 0x142;
const uint32_t Motor_MG5010_LF_ID = 0x143;
const uint32_t Motor_MG5010_RF_ID = 0x144;
const uint32_t Motor_MG5010_LB_ID = 0x145;
const uint32_t Motor_MG5010_RB_ID = 0x146;

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

void Motor::init() {
    run();
    setTorque(0);
    setSpeed(0);
    setAcceleration(0);
    readPIDParam();
    readEncoder();
    readMultiLoopsAngle();
    readSingleLoopAngle();
    readState1();
    readState2();
    readState3();
    uint8_t send_count = 11, receive_count = 0;
    while(receive_count < send_count){
        if (ESP32Can.readFrame(rxFrame, 1000)) {
            // Serial.print("ID: ");
            // Serial.print(rxFrame.identifier, HEX);
            // Serial.print(" Data: ");
            // for(uint8_t i = 0; i < rxFrame.data_length_code; i++){
            //     Serial.print(rxFrame.data[i], HEX);
            //     Serial.print(" ");
            // }
            if (updateMotor(rxFrame)) {
                receive_count++;
                break;
            }
        }
    }
    Serial.printf("Motor %x init success", _id);
}

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

void Motor::run() {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = runMotorMSG[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Run motor msg sent failed");
        delay(1);
    };
    Serial.println("Run motor msg sent success");
}

void Motor::close() {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = closeMotorMsg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Close motor msg sent failed");
        delay(1);
    };
    Serial.println("Close motor msg sent success");
}

void Motor::stop() {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = stopMotorMsg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Stop motor msg sent failed");
        delay(1);
    };
    Serial.println("Stop motor msg sent success");
}

void Motor::setTorque(int16_t iq) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data = MotorTorqueClosedControl(iq);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Torque control msg sent failed");
        delay(1);
    };
    Serial.println("Torque control msg sent success");
}

void Motor::setSpeed(int32_t speed) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data = MotorSpeedClosedControl(speed);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Speed control msg sent failed");
        delay(1);
    };
    Serial.println("Speed control msg sent success");
}

void Motor::setAngleMultiLoops(int32_t angle, uint16_t maxSpeed) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data = MotorMultiLoopsAngleClosedControl2(angle, maxSpeed);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Multi-Loops Angle control msg sent failed");
        delay(1);
    };
    Serial.println("Multi-Loops Angle control msg sent success");
}

void Motor::setAngleSingleLoop(uint32_t angle, uint16_t maxSpeed,
                               bool spinDirection) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data =
        MotorSingleLoopAngleClosedControl2(angle, maxSpeed, spinDirection);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Single-Loop Angle control msg sent failed");
        delay(1);
    };
    Serial.println("Single-Loop Angle control msg sent success");
}

void Motor::setAngleIncremental(int32_t angleIncrement, uint16_t maxSpeed) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data =
        MotorIncrementalAngleClosedControl2(angleIncrement, maxSpeed);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Incremental Angle control msg sent failed");
        delay(1);
    };
    Serial.println("Incremental Angle control msg sent success");
}

void Motor::setPID(uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp,
                   uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data = MotorWritePIDParamToROM(angle_kp, angle_ki, speed_kp,
                                            speed_ki, iq_kp, iq_ki);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("PID control msg sent failed");
        delay(1);
    };
    Serial.println("PID control msg sent success");
}

void Motor::setAcceleration(int32_t acceleration) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data = MotorWriteAccelerationToRAM(acceleration);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Acceleration control msg sent failed");
        delay(1);
    };
    Serial.println("Acceleration control msg sent success");
}

void Motor::setEncoderOffset(uint16_t encoderOffset) {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    uint8_t *data = MotorWriteEncoderToROM(encoderOffset);
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Encoder Offset control msg sent failed");
        delay(1);
    };
    Serial.println("Encoder Offset control msg sent success");
}

void Motor::readPIDParam() {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadPIDParamMsg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read PID Param msg sent failed");
        delay(1);
    };
    Serial.println("Read PID Param msg sent success");
}

void Motor::readEncoder() {
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadEncoderMsg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read Encoder msg sent failed");
        delay(1);
    };
    Serial.println("Read Encoder msg sent success");
}

void Motor::readMultiLoopsAngle(){
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadMultiLoopsAngleMsg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read Multi-Loops Angle msg sent failed");
        delay(1);
    };
    Serial.println("Read Multi-Loops Angle msg sent success");
}

void Motor::readSingleLoopAngle(){
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadSingleLoopAngleMsg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read Single-Loop Angle msg sent failed");
        delay(1);
    };
    Serial.println("Read Single-Loop Angle msg sent success");
}

void Motor::readState1(){
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadStatus1Msg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read State1 msg sent failed");
        delay(1);
    };
    Serial.println("Read State1 msg sent success");
}

void Motor::readState2(){
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadStatus2Msg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read State2 msg sent failed");
        delay(1);
    };
    Serial.println("Read State2 msg sent success");
}

void Motor::readState3(){
    CanFrame frame;
    frame.identifier = _id;
    frame.extd = false;
    frame.data_length_code = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = MotorReadStatus3Msg[i];
    }
    while (!ESP32Can.writeFrame(frame)) {
        Serial.println("Read State3 msg sent failed");
        delay(1);
    };
    Serial.println("Read State3 msg sent success");
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

bool updateMotor(const CanFrame frame) {
    MsgState state = MotorCanCommandByteJudge(frame.data[0]);
    if (state == Error) {
        Serial.println("Error: Can't match the state");
        return false;
    }
    Motor *motor = getMotorByFrameID(frame.identifier);
    if (motor == nullptr) {
        Serial.println("Error: Can't match the motor");
        return false;
    }

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
        int64_t angle = (static_cast<int64_t>(frame.data[7]) << 48) |
                        (static_cast<int64_t>(frame.data[6]) << 40) |
                        (static_cast<int64_t>(frame.data[5]) << 32) |
                        (static_cast<int64_t>(frame.data[4]) << 24) |
                        (static_cast<int64_t>(frame.data[3]) << 16) |
                        (static_cast<int64_t>(frame.data[2]) << 8) |
                        static_cast<int64_t>(frame.data[1]);
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
        return false;
        break;
    }
    }
    return true;
}

void MotorInit(){
    Motor_MF9025_L.init();
    Motor_MF9025_R.init();
    Motor_MG5010_LF.init();
    Motor_MG5010_RF.init();
    Motor_MG5010_LB.init();
    Motor_MG5010_RB.init();
}
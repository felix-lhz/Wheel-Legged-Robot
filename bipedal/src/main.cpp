#include <Arduino.h>
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "SF_Servo.h"
#include "bipedal_data.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "sbus.h"
// tockn
#include "MPU6050_tockn.h"

#define PRINTINFO

SF_Servo servos = SF_Servo(Wire); // 实例化舵机
// tockn
MPU6050 mpu6050(Wire, 0.03, 0.97);
bfs::SbusRx sbusRx(&Serial1); // 实例化接收机
SF_BLDC motors = SF_BLDC(Serial2);

#define _constrain(amt, low, high)                                             \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

void getRCValue();
void getMPUValue();
void getMotorValue();
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear,
                   uint16_t servoRightFront, uint16_t servoRightRear);
void setRobotparam();
void robotRun();
void inverseKinematics();
void legControl();
float selfCaliCentroid(float central);

std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;
robotposeparam robotPose;
robotmotionparam robotMotion;
robotmode robotMode;
motorstatus motorStatus;
controlparam controlTarget;
coordinate coordTarget;
IKparam IKParam;
motorstarget motorsTarget;
float robotLastHeight;
int RCLastCH3Value;
int RCLastCH2Value;
SF_BLDC_DATA BLDCData;

uint32_t currTime;
uint32_t prevTime;

PIDIncrement PID_Roll{.Kp = 0, .Ki = 0, .Kd = 0};
PIDIncrement PID_Gyrox{.Kp = 0, .Ki = 0, .Kd = 0};
PIDIncrement PID_Hegiht{.Kp = 0.15, .Ki = 0, .Kd = 0};
PIDIncrement PID_Y{.Kp = 0.4, .Ki = 0, .Kd = 0};
PIDIncrement PID_XCoord{.Kp = 0.34, .Ki = 0, .Kd = 0};
PIDIncrement PID_Stb{.Kp = 0, .Ki = 0, .Kd = 0};
PIDIncrement PID_Streeing{.Kp = 0.05, .Ki = 0, .Kd = 0};
PIDIncrement PID_Forward{.Kp = -1.1, .Ki = 0, .Kd = 0};
LowPassFilter LPFPitch{0.03}; // 速度低通滤波
LowPassFilter LPFRoll{0.1};
float GyroXPModify;

PIDController PID_VEL{0, 0, 0, 1000, 50};

void setup() {
    Serial.begin(921600);
    setRobotparam();
    Wire.begin(1, 2, 400000UL);
    // tockn
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    servos.init();
    sbusRx.Begin(SBUSPIN, -1);
    motors.init();
    motors.setModes(4, 4);

    delay(1000);
    currTime = micros();
}

uint8_t cnt;
uint32_t now_time;
uint32_t last_time;

void loop() {
    getRCValue();
    getMPUValue();
    getMotorValue();

    legControl();
    inverseKinematics();
    robotRun();
    last_time = now_time;
    now_time = micros();
    float Ts = (now_time - last_time) * 1e-6f;
    if (cnt++ > 50) {
        cnt = 0;
        // Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f \n", pid_vel.P, pid_vel.I,
        // pid_vel.D, PID_Stb.Kp, PID_Stb.Kd, voltage_control, OutPut_left, Ts);
        // Serial.printf("%f,%f,%f,%f \n", controlTarget.velocity,
        // controlTarget.centerAngleOffset, robotPose.speedAvg, Ts);
        // Serial.printf("M0角度:%f,M1角度%f
        // \r\n",driver.M0_angle,driver.M1_angle);
        // //检测是否收到电机主控回传的信息 Serial.printf("%d,%d,%d,%d \n",
        // Coordinate.angle_2, Coordinate.angle_1, Coordinate.angle_3,
        // Coordinate.angle_4);
    }
}

void setRobotparam() {
    // 设置机体的运动限制
    robotMotion.heightest = ROBOT_HIGHEST;
    robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
    robotMotion.forwardLimit = 30;
    robotMotion.rollLimit = 20;

    robotMode.motorEnable = false;
    robotMode.servoEnable = false;
    robotMode.printFlag = false;
    robotMode.mode = ROBOTMODE_DIABLE;

    motorStatus.M0Dir = 1;
    motorStatus.M1Dir = 1;
}

void getRCValue() {
    if (sbusRx.Read()) {
        sbusData = sbusRx.ch();
        RCValue[0] = sbusData[0];
        RCValue[1] = sbusData[1];
        RCValue[2] = sbusData[2];
        RCValue[3] = sbusData[3];
        RCValue[4] = sbusData[4];
        RCValue[5] = sbusData[5];

        RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
        RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
        RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
        RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);

#ifdef PRINTINFO
        // Serial.printf("%d,%d,%d,%d,%d,%d\n",RCValue[0],RCValue[1],RCValue[2],RCValue[3],RCValue[4],RCValue[5]);//右左右
        // 右上下 左上下 左左右
#endif // DEBUG
    }
}

void getMPUValue() {
    mpu6050.update();
    // tockn
    robotPose.pitch = -mpu6050.getAngleX(); // 摆放原因导致调换
    robotPose.roll = mpu6050.getAngleY();   // 摆放原因导致调换
    robotPose.yaw = mpu6050.getAngleZ();
    robotPose.GyroX = mpu6050.getGyroY();
    robotPose.GyroY = -mpu6050.getGyroX();
    robotPose.GyroZ = -mpu6050.getGyroZ();
#ifdef PRINTINFO

    // Serial.printf("%f,%f,\n",robotPose.pitch,robotPose.roll);
    // Serial.printf("%f,%f,%f,%f,%f,",robotPose.pitch,robotPose.roll,robotPose.GyroX,robotPose.GyroY,robotPose.GyroZ);
#endif // DEBUG
}

void getMotorValue() {
    BLDCData = motors.getBLDCData();
    motorStatus.M0Speed = BLDCData.M0_Vel;
    motorStatus.M1Speed = BLDCData.M1_Vel;
#ifdef PRINTINFO
    // Serial.printf("%f,%f\n",BLDCData.M0_Vel,BLDCData.M1_Vel);
#endif // DEBUG
}

void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear,
                   uint16_t servoRightFront, uint16_t servoRightRear) {
    servos.setPWM(LFSERVO_CH, 0, servoLeftFront);
    servos.setPWM(LRSERVO_CH, 0, servoLeftRear);
    servos.setPWM(RFSERVO_CH, 0, servoRightFront);
    servos.setPWM(RRSERVO_CH, 0, servoRightRear);
}

void legControl() {
    float e_H;
    float E_H;

    // 根据RC 设定足轮的运动参数
    robotMotion.turn = map(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX,
                           -1 * robotMotion.turnLimit, robotMotion.turnLimit);
    robotMotion.forward =
        map(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX,
            -1 * robotMotion.forwardLimit, robotMotion.forwardLimit);
    if (robotMotion.forward >= 20)
        robotMotion.forward = 20; // 正数是车身前倾，腿向后
    else if (robotMotion.forward <= -15)
        robotMotion.forward = -15; // 负数是车身后倾，腿向前

    robotMotion.updown = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX,
                                   robotMotion.lowest, robotMotion.heightest));
    robotMotion.roll = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX,
                           -1 * robotMotion.rollLimit, robotMotion.rollLimit);

    // 根据RC 设定足轮的电机与舵机使能状态
    if (RCValue[4] == RCCHANNEL3_MIN) {
        robotMode.motorEnable = true;
        robotMode.servoEnable = true;
    } else if (RCValue[4] == RCCHANNEL3_MID) {
        // robotMode.motorEnable = true;
        // robotMode.servoEnable = false;
        robotMode.motorEnable = false;
        robotMode.servoEnable = true;
    } else if (RCValue[4] == RCCHANNEL3_MAX) {
        robotMode.motorEnable = false;
        robotMode.servoEnable = false;
    }

    // 根据RC 设定足轮的控制模式
    if (RCValue[5] == RCCHANNEL3_MIN) {
        robotMode.mode = ROBOTMODE_MOTION;
        robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
        robotMode.printFlag = false;
    } else if (RCValue[5] == RCCHANNEL3_MID) {
        robotMode.printFlag = true;
    } else if (RCValue[5] == RCCHANNEL3_MAX) {
        robotMode.mode = ROBOTMODE_CALIBRATION;
        robotMotion.forward = 0;
        robotMotion.updown = ROBOT_LOWESR_FOR_CAL;
        robotMotion.lowest = ROBOT_LOWESR_FOR_CAL;
        robotMode.printFlag = false;
    }

    if (abs(RCValue[2] - RCLastCH3Value) >= 5) {
        GyroXPModify = 0.3f;
        RCLastCH3Value = RCValue[2];
    } else {
        GyroXPModify = 0.5f;
    }

    controlTarget.forward =
        PID_Forward.Kp *
        (robotMotion.forward - (motorStatus.M0Dir * motorStatus.M0Speed +
                                motorStatus.M1Dir * motorStatus.M1Speed) /
                                   2);
    controlTarget.forward = _constrain(controlTarget.forward, -20, 20);
    // coordTarget.x = coordTarget.x + PID_XCoord.Kp*(controlTarget.forward -
    // coordTarget.x);

    if (robotMode.mode == ROBOTMODE_MOTION && robotMode.servoEnable == true) {

        e_H = PID_Roll.Kp * LPFRoll((robotMotion.roll - 3) - robotPose.roll);
        E_H = PID_Gyrox.Kp * (e_H - robotPose.GyroX);
    } else {
        e_H = 0;
        E_H = 0;
    }
    // Serial.printf("%f,%f\n",e_H,E_H);

    controlTarget.legLeft =
        controlTarget.legLeft +
        PID_Hegiht.Kp * (robotMotion.updown - controlTarget.legLeft);
    controlTarget.legRight =
        controlTarget.legRight +
        PID_Hegiht.Kp * (robotMotion.updown - controlTarget.legRight);

    controlTarget.legRollLeft =
        _constrain(controlTarget.legRollLeft + E_H, -robotMotion.lowest,
                   robotMotion.lowest);
    controlTarget.legRollRight =
        _constrain(controlTarget.legRollRight - E_H, -robotMotion.lowest,
                   robotMotion.lowest);

    coordTarget.yLeft = _constrain(
        coordTarget.yLeft +
            PID_Y.Kp * ((robotMotion.updown + controlTarget.legRollLeft) -
                        coordTarget.yLeft),
        robotMotion.lowest, robotMotion.heightest);
    coordTarget.yRight = _constrain(
        coordTarget.yRight +
            PID_Y.Kp * ((robotMotion.updown + controlTarget.legRollRight) -
                        coordTarget.yRight),
        robotMotion.lowest, robotMotion.heightest);

    coordTarget.yLeft =
        _constrain(controlTarget.legRollLeft + controlTarget.legLeft,
                   robotMotion.lowest, robotMotion.heightest);
    coordTarget.yRight =
        _constrain(controlTarget.legRollRight + controlTarget.legRight,
                   robotMotion.lowest, robotMotion.heightest);

    coordTarget.x =
        coordTarget.x + PID_XCoord.Kp * (controlTarget.forward - coordTarget.x);

    robotPose.height = (coordTarget.yLeft + coordTarget.yRight) / 2;
}

void inverseKinematics() {
    coordTarget.xLeft = coordTarget.x;
    if (coordTarget.xLeft == 0)
        coordTarget.xRight = coordTarget.xLeft + 34;
    else if (coordTarget.xLeft > 0)
        coordTarget.xRight = 34 - coordTarget.xLeft;
    else if (coordTarget.xLeft < 0)
        coordTarget.xRight = 34 - coordTarget.xLeft;

    float a1 = 2 * coordTarget.xLeft * L1;
    float b1 = 2 * coordTarget.yLeft * L1;
    float c1 = coordTarget.xLeft * coordTarget.xLeft +
               coordTarget.yLeft * coordTarget.yLeft + L1 * L1 -
               (L2 + L6) * (L2 + L6);

    IKParam.alpha11 =
        2 * atan((b1 + sqrt((a1 * a1) + (b1 * b1) - (c1 * c1))) / (a1 + c1));

    IKParam.XbLeft =
        coordTarget.xLeft -
        L6 * ((coordTarget.xLeft - L1 * cos(IKParam.alpha11)) / (L2 + L6));
    IKParam.YbLeft =
        coordTarget.yLeft -
        L6 * ((coordTarget.yLeft - L1 * sin(IKParam.alpha11)) / (L2 + L6));

    float d1 = 2 * L4 * (IKParam.XbLeft - L5);
    float e1 = 2 * L4 * IKParam.YbLeft;
    float f1 = ((IKParam.XbLeft - L5) * (IKParam.XbLeft - L5) + L4 * L4 +
                IKParam.YbLeft * IKParam.YbLeft - L3 * L3);

    IKParam.alpha12 =
        2 * atan((e1 - sqrt((d1 * d1) + e1 * e1 - (f1 * f1))) / (d1 + f1));

    // 限制解算角度的范围
    if (IKParam.alpha11 < 0)
        IKParam.alpha11 = IKParam.alpha11 + 2 * PI;

    if (IKParam.alpha12 < 0)
        IKParam.alpha12 = 0;

    float alpha11ToAngle = (IKParam.alpha11 / 6.28) * 360; // 弧度转角度
    float alpha12ToAngle = (IKParam.alpha12 / 6.28) * 360;

    // motorsTarget.servoLeftFront = (int)map(alpha11ToAngle, 0, 180, 103, 327);
    // // 1号舵机 500~1500us(500~2500) motorsTarget.servoLeftRear =
    // (int)map(alpha12ToAngle, 0, 180, 103, 327);  // 2号舵机

    motorsTarget.servoLeftRear = (int)map(alpha11ToAngle, 0, 180, 103,
                                          327); // 1号舵机 500~1500us(500~2500)
    motorsTarget.servoLeftFront =
        (int)map(alpha12ToAngle, 0, 180, 103, 327); // 2号舵机

    float a2 = 2 * coordTarget.xRight * L1;
    float b2 = 2 * coordTarget.yRight * L1;
    float c2 = coordTarget.xRight * coordTarget.xRight +
               coordTarget.yRight * coordTarget.yRight + L1 * L1 -
               (L2 + L6) * (L2 + L6);

    IKParam.alpha21 =
        2 * atan((b2 + sqrt((a2 * a2) + (b2 * b2) - (c2 * c2))) / (a2 + c2));

    IKParam.XbRight =
        coordTarget.xRight -
        L6 * ((coordTarget.xRight - L1 * cos(IKParam.alpha21)) / (L2 + L6));
    IKParam.Ybright =
        coordTarget.yRight -
        L6 * ((coordTarget.yRight - L1 * sin(IKParam.alpha21)) / (L2 + L6));

    float d2 = 2 * L4 * (IKParam.XbRight - L5);
    float e2 = 2 * L4 * IKParam.Ybright;
    float f2 = ((IKParam.XbRight - L5) * (IKParam.XbRight - L5) + L4 * L4 +
                IKParam.Ybright * IKParam.Ybright - L3 * L3);

    IKParam.alpha22 =
        2 * atan((e2 - sqrt((d2 * d2) + e2 * e2 - (f2 * f2))) / (d2 + f2));

    if (IKParam.alpha21 < 0)
        IKParam.alpha21 = IKParam.alpha21 + 2 * PI;
    // IKParam.alpha1=_constrain(IKParam.alpha1,0,3.1);

    if (IKParam.alpha22 < 0)
        // IKParam.alpha12 = IKParam.alpha12 + 2 * PI;
        IKParam.alpha22 = 0;

    float alpha21ToAngle = (IKParam.alpha21 / 6.28) * 360 + 4; // todo
    float alpha22ToAngle = (IKParam.alpha22 / 6.28) * 360 - 2;

    motorsTarget.servoRightFront = (int)map(
        alpha21ToAngle, 0, 180, 103, 327); // 1号舵机 500~1500us(500~2500)
    motorsTarget.servoRightRear =
        (int)map(alpha22ToAngle, 0, 180, 103, 327); // 2号舵机

    // motorsTarget.servoRightRear = (int)map(alpha21ToAngle, 0, 180, 103, 327);
    // // 1号舵机 500~1500us(500~2500) motorsTarget.servoRightFront =
    // (int)map(alpha22ToAngle, 0, 180, 103, 327);  // 2号舵机

    if (robotMode.servoEnable) {
        setServoAngle(motorsTarget.servoLeftFront, motorsTarget.servoLeftRear,
                      motorsTarget.servoRightFront,
                      motorsTarget.servoRightRear);
        Serial.printf("%d,%d,%d,%d\n", motorsTarget.servoLeftFront,
                      motorsTarget.servoLeftRear, motorsTarget.servoRightFront,
                      motorsTarget.servoRightRear);
    }
}

void robotRun() {
    float velLeft, velRight;
    float wheelControl;

    if (robotPose.height != robotLastHeight) {
        PID_VEL.P = (0.00002 * robotPose.height * robotPose.height -
                     0.007 * robotPose.height + 0.669) *
                    1.8;
        PID_VEL.I = 0.00153 * 0.6; // 1
        PID_VEL.D = (0.0000002 * robotPose.height * robotPose.height -
                     0.00001 * robotPose.height - 0.01) *
                    0.1;

        PID_Stb.Kp = (0.0003 * robotPose.height * robotPose.height -
                      0.0488 * robotPose.height + 3.5798) *
                     0.8; // 0.7
        PID_Stb.Kd = (-0.000002 * robotPose.height * robotPose.height +
                      0.0005 * robotPose.height - 0.0043) *
                     1.6; // 1.6

        PID_Roll.Kp = (0.001 * robotPose.height * robotPose.height -
                       0.2281 * robotPose.height + 17.495) *
                      1.3;

        PID_Gyrox.Kp = (0.0000009 * robotPose.height * robotPose.height -
                        0.0005 * robotPose.height + 0.091) *
                       GyroXPModify;

        robotMotion.turnLimit =
            (0.000009 * robotPose.height * robotPose.height -
             0.005 * robotPose.height + 120.5104) *
            1;

        robotLastHeight = robotPose.height;
    }

    robotPose.speedAvg = (motorStatus.M0Dir * motorStatus.M0Speed +
                          motorStatus.M1Dir * motorStatus.M1Speed) /
                         2;

    if (RCLastCH2Value == RCValue[1]) {
        controlTarget.centerAngleOffset =
            selfCaliCentroid(controlTarget.centerAngleOffset);
    }
    RCLastCH2Value = RCValue[1];

    controlTarget.velocity =
        PID_VEL(robotMotion.forward * 0.3f - robotPose.speedAvg); // 速度环
    controlTarget.differVel =
        PID_Streeing.Kp * (robotMotion.turn - robotPose.GyroZ); // 转向
    float targetVoltage =
        PID_Stb.Kp * (controlTarget.velocity + controlTarget.centerAngleOffset -
                      robotPose.pitch) -
        PID_Stb.Kd * robotPose.GyroY; // 直立环 输出控制的电机的目标电压

    motorsTarget.motorLeft =
        motorStatus.M0Dir * (targetVoltage + controlTarget.differVel);
    motorsTarget.motorRight =
        -motorStatus.M1Dir * (targetVoltage - controlTarget.differVel);

    if (robotMode.motorEnable == 1 && robotPose.pitch <= 40 &&
        robotPose.pitch >= -35) {
        motorsTarget.motorLeft = _constrain(motorsTarget.motorLeft, -5.7, 5.7);
        motorsTarget.motorRight =
            _constrain(motorsTarget.motorRight, -5.7, 5.7);
        motors.setTargets(motorsTarget.motorLeft, motorsTarget.motorRight);
    } else {
        motors.setTargets(0, 0);
    }
}

float selfCaliGain = 0.5;
float selfcaliOffset = 0;
#define SELF_CALI_RANGE 7
#define CENTER_ANGLE_OFFSET 3
float selfCaliCentroid(float central) {
    static int i = 0;
    if (i == 40) {
        if (fabs(robotPose.speedAvg) > 1) {
            selfcaliOffset = selfCaliGain * -1 * robotPose.speedAvg;
            selfcaliOffset = _constrain(selfcaliOffset, -0.5, 0.5);
            central += selfcaliOffset;
        }
        i = 0;
    } else {
        ++i;
    }
    central = _constrain(central, CENTER_ANGLE_OFFSET - SELF_CALI_RANGE,
                         CENTER_ANGLE_OFFSET + SELF_CALI_RANGE);

    return central;
}

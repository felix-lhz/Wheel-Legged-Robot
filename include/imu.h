#ifndef IMU_H
#define IMU_H

#include "I2Cdev.h"
#include "IIC.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "calc.h"
#include "im948_CMD.h"

class IMU {
  private:
    MPU6050 imu;
    float euler[3]; // [psi, theta, phi]    Euler angle container
    float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity
                    // vector
    Quaternion q;   // [w, x, y, z]         quaternion container
    VectorInt16 aa; // [x, y, z]            accel sensor measurements
    VectorInt16
        aaReal; // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16
        aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t interrupt_pin;
    int16_t temperature;

    // MPU control/status vars
    bool dmpReady = false; // set true if DMP init was successful

  public:
    IMU() {};
    IMU(uint8_t _scl_pin, uint8_t _sda_pin, uint8_t _interrupt_pin);
    ~IMU() {};
    void update();

    float getYawRad();
    float getPitchRad();
    float getRollRad();
    float getPsiRad();
    float getThetaRad();
    float getPhiRad();

    float getYawDeg();
    float getPitchDeg();
    float getRollDeg();
    float getPsiDeg();
    float getThetaDeg();
    float getPhiDeg();

    Quaternion getQuaternion();
    VectorInt16 getSensorAccel();
    VectorInt16 getRealAccel();
    VectorInt16 getWorldAccel();
    VectorFloat getGravity();

    int16_t getTemperature();
};

class WitImu {
  private:
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    IIC iic;
    uint8_t sda;
    uint8_t scl;
    uint8_t device_addr;
    const uint8_t angle_start_addr = 0x3d;
    const uint8_t unlock_addr = 0x69;
    uint8_t unlock_data[2] = {0x88, 0xB5};
    const uint8_t save_addr = 0x00;
    uint8_t save_data[2] = {0x00, 0x00};
    const uint8_t cal_addr = 0x01;
    uint8_t cal_gyro_acc[2] = {0x01, 0x00};  // 陀螺仪和加速度计校准
    uint8_t cal_altitude[2] = {0x03, 0x00};  // 高度计校准
    uint8_t cal_z_angle[2] = {0x04, 0x00};   // Z轴角度校准
    uint8_t cal_ref_angle[2] = {0x08, 0x00}; // 参考角度校准

  public:
    WitImu() {};
    WitImu(uint8_t _sda, uint8_t _scl, uint8_t _device_addr);
    ~WitImu() {};
    void init();
    void calibrate(uint8_t *cmd);
    void update();
    double getRoll() { return roll; }
    double getPitch() { return pitch; }
    double getYaw() { return yaw; }
};

class IMU948 {
  private:
    int8_t rx = 19;
    int8_t tx = 20;

    bool first_flag = true;

    float angle_x = 0;
    float angle_y = 0;
    float angle_z = 0;
    float a_x = 0;
    float a_y = 0;
    float a_z = 0;
    float g_x = 0;
    float g_y = 0;
    float g_z = 0;
    float temperature = 0;
    float air_press = 0;
    float height = 0;
    
    float reference_height = 0;

  public:
    // default pin is rx:19,tx:20
    IMU948() {};
    IMU948(int8_t rx, int8_t tx);
    ~IMU948() {};
    void init();
    void update();

    float getAngleX() { return angle_x; }
    float getAngleY() { return angle_y; }
    float getAngleZ() { return angle_z; }
    float getAccelX() { return a_x; }
    float getAccelY() { return a_y; }
    float getAccelZ() { return a_z; }
    float getGyroX() { return g_x; }
    float getGyroY() { return g_y; }
    float getGyroZ() { return g_z; }
    float getTemperature() { return temperature; }
    float getAirPress() { return air_press; }
    float getHeight() { return height; }
};

#endif
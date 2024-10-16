#ifndef IMU_H
#define IMU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "calc.h"

class IMU {
    private:
        MPU6050 imu;
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        uint8_t sda_pin;
        uint8_t scl_pin;
        uint8_t interrupt_pin;
        int16_t temperature;

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        
    public:
        IMU(){};
        IMU(uint8_t _scl_pin , uint8_t _sda_pin, uint8_t _interrupt_pin);
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

#endif
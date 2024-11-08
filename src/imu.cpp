#include "imu.h"

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

IMU::IMU(uint8_t _scl_pin , uint8_t _sda_pin, uint8_t _interrupt_pin) {
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

    sda_pin = _sda_pin;
    scl_pin = _scl_pin;
    interrupt_pin = _interrupt_pin;

    //initialize I2C
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(400000);

    // initialize device
    Serial.println("Initializing I2C devices...");
    imu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(imu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    pinMode(interrupt_pin, INPUT);
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = imu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    imu.setXGyroOffset(220);
    imu.setYGyroOffset(76);
    imu.setZGyroOffset(-85);
    imu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        imu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        imu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(_interrupt_pin));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(_interrupt_pin), dmpDataReady, RISING);
        mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void IMU::update() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    uint8_t fifoBuffer[64]; // FIFO storage buffer

    if (imu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        
        // display quaternion values in easy matrix form: w x y z
        imu.dmpGetQuaternion(&q, fifoBuffer);

        // display Euler angles in degrees
        imu.dmpGetEuler(euler, &q);

        // display Euler angles in degrees
        imu.dmpGetGravity(&gravity, &q);
        imu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // display real acceleration, adjusted to remove gravity
        imu.dmpGetAccel(&aa, fifoBuffer);
        imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        imu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }
    temperature = imu.getTemperature();
}

float IMU::getYawRad() {
    return ypr[0];
}

float IMU::getPitchRad() {
    return ypr[1];
}

float IMU::getRollRad() {
    return ypr[2];
}

float IMU::getPsiRad() {
    return euler[0];
}

float IMU::getThetaRad() {
    return euler[1];
}

float IMU::getPhiRad() {
    return euler[2];
}

float IMU::getYawDeg() {
    return rad2deg(ypr[0]);
}

float IMU::getPitchDeg() {
    return rad2deg(ypr[1]);
}

float IMU::getRollDeg() {
    return rad2deg(ypr[2]);
}

float IMU::getPsiDeg() {
    return rad2deg(euler[0]);
}

float IMU::getThetaDeg() {
    return rad2deg(euler[1]);
}

float IMU::getPhiDeg() {
    return rad2deg(euler[2]);
}

Quaternion IMU::getQuaternion() {
    return q;
}

VectorInt16 IMU::getSensorAccel() {
    return aa;
}

VectorInt16 IMU::getRealAccel() {
    return aaReal;
}

VectorInt16 IMU::getWorldAccel() {
    return aaWorld;
}

VectorFloat IMU::getGravity() {
    return gravity;
}

int16_t IMU::getTemperature() {
    return temperature;
}


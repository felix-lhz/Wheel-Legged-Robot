#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include "Pins_Specify.h"
#include "SF_Communication.h"
#include "SF_Motor.h"

// ���ڷ��͵�

SPIClass vspi(VSPI); // ����õ�MT6701�����������õ�SPIͨ�ţ���Ҫʵ����SPI����
TwoWire iic0 = TwoWire(
    0); // �����AS5600������������IICͨ�ţ�˫���������Ҫʵ��������IIC����
TwoWire iic1 = TwoWire(1);

SF_Motor M0 = SF_Motor(0); // ʵ���������
SF_Motor M1 = SF_Motor(1);

SF_Communication com = SF_Communication(); // ʵ����ͨ�Žӿ�

float Vbus = 12.0;      // ���ù����ѹֵ
float alignVoltage = 3; // ���-������У׼ʱ�ĵ�ѹֵ

void setup() {
    Serial.begin(115200);
    com.linkMotor(M0, M1); // ���ӵ����ͨ�Žӿ� ����ѡһ��������
    com.init(
        ONBOARD); // ѡ��ͨ�ŵ���� USB:ͨ��USB�����ONBOARD:����չ����ͨ�š�

    // ΪAS5600�������ĳ�ʼ��
    //  iic0.begin(AS5600_SDA0, AS5600_SCL0, 400000UL);
    //  iic1.begin(AS5600_SDA1, AS5600_SCL1, 400000UL);
    //  M0.initEncoder(AS5600,iic0);
    //  M1.initEncoder(AS5600,iic1);
    // ΪMT6701�������ĳ�ʼ��
    vspi.begin(MT6701_CLK, MT6701_DO, 0, -1);
    M0.initEncoder(MT6701, vspi);
    M1.initEncoder(MT6701, vspi);

    // �����ʼ���������ѹֵ
    M0.init(Vbus);
    M1.init(Vbus);
    // ���-��������У׼������У׼ʱ�ĵ�ѹֵ
    M0.AlignSensor(alignVoltage);
    M1.AlignSensor(alignVoltage);

    // ͨ�Ŷ˿ڿ��������ĵ��״̬���
    com.start();

    // ������ص�PIDֵ �������ΪP��I��D��Limitֵ
    M0.setAnglePID(0.5, 0, 0, 6);
    M0.setVelPID(0.05, 0.005, 0, 6);
    M0.setCurrentPID(1.2, 0, 0, 0);

    M1.setAnglePID(0.5, 0, 0, 6);
    M1.setVelPID(0.05, 0.005, 0, 6);
    M1.setCurrentPID(1.2, 0, 0, 0);
}

uint32_t now_time = 0;
uint32_t last_time = 0;

void loop() {
    // �����ѭ������ִ��
    M0.run();
    M1.run();

    // ���õ���Ŀ��Ʒ�ʽʾ��,��ȻҲ����ͨ�����ڿ���
    //  M0.setTorque(1);
    //  M1.setTorque(1);
    //  M0.setForceAngle(0);
    //  M1.setForceAngle(0);

    // M0.setVelocity(20);
    // M1.setVelocity(20);
}

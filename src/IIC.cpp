#include "IIC.h"

IIC::IIC(uint8_t _sda_pin, uint8_t _scl_pin, uint8_t _device_address) {
    _sda = _sda_pin;
    _scl = _scl_pin;
    device_addr = _device_address;
    pinMode(_sda, OUTPUT);
    pinMode(_scl, OUTPUT);
    digitalWrite(_sda, HIGH);
    digitalWrite(_scl, HIGH);
}

void IIC::begin() {
    SDA_H();
    SCL_H();
    delayMicroseconds(5);
    SDA_L();
    delayMicroseconds(5);
    SCL_L();
    delayMicroseconds(2);
}

void IIC::stop() {
    SCL_L();
    SDA_L();
    delayMicroseconds(5);
    SCL_H();
    delayMicroseconds(5);
    SDA_H();
    delayMicroseconds(2);
}

uint8_t IIC::wait_ack() {
    uint8_t error_time = 0;
    SDA_H();
    delayMicroseconds(2);
    SCL_H();
    delayMicroseconds(2);

    pinMode(_sda, INPUT);
    while (digitalRead(_sda)) {
        error_time++;
        if (error_time > 250) {
            stop();
            return 1;
        }
    }
    SCL_L();
    delayMicroseconds(2);
    return 0;
}

void IIC::send_ack() {
    SCL_L();
    SDA_L();
    delayMicroseconds(2);
    SCL_H();
    delayMicroseconds(5);

    SCL_L();
    delayMicroseconds(2);
    SDA_H();
    delayMicroseconds(2);
}

void IIC::send_nack() {
    SCL_L();
    SDA_H();
    delayMicroseconds(2);
    SCL_H();
    delayMicroseconds(5);

    SCL_L();
    delayMicroseconds(2);
    SDA_H();
    delayMicroseconds(2);
}

void IIC::send_byte(uint8_t data) {
    uint8_t i = 0;
    for (i = 0; i < 8; i++) {
        SCL_L();
        if (data & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        delayMicroseconds(5);
        SCL_H();
        delayMicroseconds(5);
        SCL_L();
        data <<= 1;
        delayMicroseconds(5);
    }
    delayMicroseconds(2);
    SDA_H();
    delayMicroseconds(2);
}

uint8_t IIC::read_byte(uint8_t ack) {
    uint8_t i = 0;
    uint8_t data = 0;
    delayMicroseconds(2);
    SDA_H();
    delayMicroseconds(2);

    for (i = 0; i < 8; i++) {
        SCL_L();
        delayMicroseconds(5);
        SCL_H();
        data <<= 1;
        delayMicroseconds(5);
        pinMode(_sda, INPUT);
        if (digitalRead(_sda)) {
            data |= 0x01;
        } else {
            data &= 0xfe;
        }
        delayMicroseconds(5);
    }
    if (ack) {
        send_ack();
    } else {
        send_nack();
    }
    SCL_L();
    delayMicroseconds(2);
    return data;
}

uint8_t IIC::write_reg(uint8_t addr, uint8_t *data, uint32_t length) {
    begin();
    send_byte(device_addr << 1);
    if (wait_ack() == 1) {
        return 0;
    }
    send_byte(addr);
    if (wait_ack() == 1) {
        return 0;
    }

    for (uint32_t i = 0; i < length; i++) {
        send_byte(data[i]);
        if (wait_ack() == 1) {
            return 0;
        }
    }

    stop();

    return 1;
}

uint8_t IIC::read_reg(uint8_t addr, uint8_t *data, uint32_t length) {
    begin();
    send_byte((device_addr << 1) | 0);
    if (wait_ack() == 1) {
        return 0;
    }
    send_byte(addr);
    if (wait_ack() == 1) {
        return 0;
    }
    delayMicroseconds(5);

    begin();
    send_byte(device_addr | 0x01);
    if (wait_ack() == 1) {
        return 0;
    }
    for (uint32_t i = 0; i < length; i++) {
        data[i] = read_byte(i == length - 1 ? 1 : 0);
    }
    stop();
    return 1;
}

void IIC::SDA_H() {
    pinMode(_sda, OUTPUT);
    digitalWrite(_sda, HIGH);
}

void IIC::SDA_L() {
    pinMode(_sda, OUTPUT);
    digitalWrite(_sda, LOW);
}

void IIC::SCL_H() { digitalWrite(_scl, HIGH); }

void IIC::SCL_L() { digitalWrite(_scl, LOW); }

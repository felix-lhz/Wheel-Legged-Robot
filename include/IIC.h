#ifndef IIC_H
#define IIC_H
#include <Arduino.h>

// GPIO 模拟 IIC
class IIC {
  private:
    uint8_t _sda;
    uint8_t _scl;
    uint8_t device_addr;

  public:
    IIC() {};
    IIC(uint8_t _sda_pin, uint8_t _scl_pin, uint8_t _device_address);
    ~IIC() {};
    void begin();
    void stop();
    uint8_t wait_ack();
    void send_ack();
    void send_nack();
    void send_byte(uint8_t data);
    uint8_t read_byte(uint8_t ack);
    uint8_t write_reg(uint8_t addr, uint8_t *data, uint32_t length);
    uint8_t read_reg(uint8_t addr, uint8_t *data, uint32_t length);
    void SDA_H();
    void SDA_L();
    void SCL_H();
    void SCL_L();
};

#endif
#ifndef __NXP_IMU_H__
#define __NXP_IMU_H__

#include <stdint.h>

#include <driver/gpio.h>
#include <driver/i2c.h>

class NXPIMU {
public:
    NXPIMU(i2c_port_t i2cPort, gpio_num_t scl, gpio_num_t sda);

    void getMeasurements();

protected:
    uint8_t readRegister(uint8_t address, uint8_t reg);

    void readRegister(uint8_t address, uint8_t reg, size_t size, uint8_t* data);

    void writeRegister(uint8_t address, uint8_t reg, uint8_t value);

private:
    i2c_port_t i2cPort;
};

#endif

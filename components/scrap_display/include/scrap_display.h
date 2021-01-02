#ifndef __SCRAP_DISPLAY_H__
#define __SCRAP_DISPLAY_H__

#include <driver/gpio.h>
#include <u8g2_esp32_hal.h>

class ScrapDisplay {
public:
    ScrapDisplay(i2c_port_t i2cPort, gpio_num_t scl, gpio_num_t sda, uint8_t address);

    void test();

private:
    i2c_port_t i2cPort;
    gpio_num_t scl;
    gpio_num_t sda;
    uint8_t address;

    u8g2_t u8g2;
};

#endif

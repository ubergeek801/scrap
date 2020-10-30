#ifndef __U8G2_ESP32_H__
#define __U8G2_ESP32_H__

#include "u8g2_esp32_hal.h"
#include "U8g2lib.h"

class U8G2_SSD1306_128X32_GENERIC_I2C: public U8G2 {
public:
	U8G2_SSD1306_128X32_GENERIC_I2C(i2c_port_t i2cPort, uint8_t i2cAddress, uint32_t clockSpeed,
			gpio_num_t sdaPin, gpio_num_t sclPin) : U8G2() {
		u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
		u8g2_esp32_hal.sda = sdaPin;
		u8g2_esp32_hal.scl = sclPin;
		u8g2_esp32_hal.i2c_port = i2cPort;
		u8g2_esp32_hal.clock_speed = clockSpeed;
		u8g2_esp32_hal_init(u8g2_esp32_hal);

		u8g2_Setup_ssd1306_i2c_128x32_univision_f(&u8g2,
				U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
		u8x8_SetI2CAddress(&u8g2.u8x8, i2cAddress);
	}
};

#endif

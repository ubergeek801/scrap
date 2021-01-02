#include <freertos/FreeRTOS.h>

#include "scrap_display.h"
#include "scrap_logo.h"

ScrapDisplay::ScrapDisplay(i2c_port_t i2cPort, gpio_num_t scl, gpio_num_t sda, uint8_t address)
        : i2cPort(i2cPort), scl(scl), sda(sda), address(address) {
    u8g2_esp32_hal_t u8g2Config = U8G2_ESP32_HAL_DEFAULT;
    u8g2Config.i2c_port = i2cPort;
    u8g2Config.clock_speed = 1000 * 1000; // spec calls for 400 KHz but 1 MHz seems fine
    u8g2Config.sda = sda;
    u8g2Config.scl = scl;
    u8g2_esp32_hal_init(u8g2Config);

    u8g2_Setup_sh1107_i2c_64x128_f(&u8g2, U8G2_R1, u8g2_esp32_i2c_byte_cb,
            u8g2_esp32_gpio_and_delay_cb);
    u8x8_SetI2CAddress(&u8g2.u8x8, address << 1); // u8 likes the I2C address shifted left

    u8g2_InitDisplay(&u8g2);
}

void ScrapDisplay::test() {
    u8g2_SetContrast(&u8g2, 0x4f); // actually brightness
    // borrow some settings from Adafruit (https://github.com/adafruit/Adafruit_SH110x) for the
    // FeatherWing 128x64 OLED; some (which?) of these help with display quality
    u8g2_SendF(&u8g2, "ca", 0xd5, 0x51); // divide ratio/oscillator frequency
    u8g2_SendF(&u8g2, "ca", 0xad, 0x8a); // DC-DC switching frequency
    u8g2_SendF(&u8g2, "ca", 0xd9, 0x22); // pre-charge period mode
    u8g2_SendF(&u8g2, "ca", 0xdb, 0x35); // VCOM deselect level
    u8g2_SendF(&u8g2, "ca", 0xa8, 0x3f); // multiplex ratio

    // this is apparently recommended after init
    vTaskDelay(100 / portTICK_PERIOD_MS);

    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawXBM(&u8g2, 0, 0, 128, 32, scrap_bits);
    u8g2_DrawXBM(&u8g2, 0, 32, 128, 32, scrap_bits);
    u8g2_SendBuffer(&u8g2);

/*
    vTaskDelay(SPLASH_SCREEN_DELAY_MS / portTICK_PERIOD_MS);

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_tom_thumb_4x6_tr);
    u8g2_SetFontPosTop(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 0, "Scale Car Racing");
    u8g2_DrawStr(&u8g2, 0, 6, "Automation Platform");
    u8g2_DrawStr(&u8g2, 0, 12, "steering");
    u8g2_DrawStr(&u8g2, 0, 18, "throttle");
    u8g2_SendBuffer(&u8g2);
    xTaskCreate(updateDisplay, "updateDisplay", 4096, NULL, 5, NULL);
*/
}

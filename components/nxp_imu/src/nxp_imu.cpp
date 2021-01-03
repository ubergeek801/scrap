#include <freertos/FreeRTOS.h>
#include <esp_log.h>

#include "nxp_imu.h"

#define FXOS8700CQ_I2C_ADDRESS 0x1E
#define FXAS21002CQ_I2C_ADDRESS 0x20

static const char* LOG = "nxpimu";

NXPIMU::NXPIMU(i2c_port_t i2cPort, gpio_num_t scl, gpio_num_t sda) : i2cPort(i2cPort) {
    ESP_ERROR_CHECK(i2c_driver_install(i2cPort, I2C_MODE_MASTER, 0, 0, 0));

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // BRKT-STBC-AGM01 has onboard pullups
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = {
            .clk_speed = 400 * 1000 // max for FXAS21002C and FXOS8700CQ
        },
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(i2cPort, &config));

    uint8_t whoami = readRegister(FXOS8700CQ_I2C_ADDRESS, 0x0D);
    ESP_LOGI(LOG, "FXOS8700 whoami=%02x", whoami); // should be 0xC7

    whoami = readRegister(FXAS21002CQ_I2C_ADDRESS, 0x0C);
    ESP_LOGI(LOG, "FXAS21002 whoami=%02x", whoami); // should be 0xD7

    // must be in standby mode to change configuration
    writeRegister(FXOS8700CQ_I2C_ADDRESS, 0x2A, 0x00);

    writeRegister(FXOS8700CQ_I2C_ADDRESS, 0x5B, 0x1F);
    writeRegister(FXOS8700CQ_I2C_ADDRESS, 0x5C, 0x20);
    writeRegister(FXOS8700CQ_I2C_ADDRESS, 0x0E, 0x01);

    // set active mode with 50Hz output data rate (hybrid) and reduced noise
    writeRegister(FXOS8700CQ_I2C_ADDRESS, 0x2A, 0x1D);

    // must be in standby mode to change configuration
    writeRegister(FXAS21002CQ_I2C_ADDRESS, 0x13, 0x00);
    // set active mode with 50Hz output data rate
    writeRegister(FXAS21002CQ_I2C_ADDRESS, 0x13, 0x12);
}

void NXPIMU::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t command = i2c_cmd_link_create();

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(command));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(command, address << 1 | I2C_MASTER_WRITE,
            I2C_MASTER_ACK));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(command, reg, I2C_MASTER_ACK));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(command, value, I2C_MASTER_ACK));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(command));

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2cPort, command, 250 / portTICK_RATE_MS));
    i2c_cmd_link_delete(command);
}

uint8_t NXPIMU::readRegister(uint8_t address, uint8_t reg) {
    uint8_t value = 0;

    readRegister(address, reg, 1, &value);

    return value;
}

void NXPIMU::readRegister(uint8_t address, uint8_t reg, size_t size, uint8_t* data) {
    i2c_cmd_handle_t command = i2c_cmd_link_create();

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(command));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(command, address << 1 | I2C_MASTER_WRITE,
            I2C_MASTER_ACK));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(command, reg, I2C_MASTER_ACK));

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(command));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(command, address << 1 | I2C_MASTER_READ,
            I2C_MASTER_ACK));
    // for all bytes but the last, send acks
    if (size > 1) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_read(command, data, size - 1, I2C_MASTER_ACK));
    }
    // for the last byte, send a nack to stop transmission
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_read_byte(command, data + size - 1, I2C_MASTER_NACK));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(command));

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2cPort, command, 250 / portTICK_RATE_MS));
    i2c_cmd_link_delete(command);
}

InertialData NXPIMU::getMeasurements() {
    InertialData inertialData;

    uint8_t accelMagData[13];
    readRegister(FXOS8700CQ_I2C_ADDRESS, 0x00, sizeof(accelMagData), accelMagData);
    inertialData.accelX = (int16_t)(((accelMagData[1] << 8) | accelMagData[2])) >> 2;
    inertialData.accelY = (int16_t)(((accelMagData[3] << 8) | accelMagData[4])) >> 2;
    inertialData.accelZ = (int16_t)(((accelMagData[5] << 8) | accelMagData[6])) >> 2;
    inertialData.magX = (accelMagData[7] << 8) | accelMagData[8];
    inertialData.magY = (accelMagData[9] << 8) | accelMagData[10];
    inertialData.magZ = (accelMagData[11] << 8) | accelMagData[12];

    uint8_t rotData[7];
    readRegister(FXAS21002CQ_I2C_ADDRESS, 0x00, sizeof(rotData), rotData);
    inertialData.rotX = (rotData[1] << 8) | rotData[2];
    inertialData.rotY = (rotData[3] << 8) | rotData[4];
    inertialData.rotZ = (rotData[5] << 8) | rotData[6];

    // 8700's temperature is uncalibrated so adjust it by an empirically determined offset
    inertialData.temp1 = (uint8_t)(readRegister(FXOS8700CQ_I2C_ADDRESS, 0x51) * 0.96) + 6;
    inertialData.temp2 = readRegister(FXAS21002CQ_I2C_ADDRESS, 0x12);

    return inertialData;
}

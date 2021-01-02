#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>

#include "adns3080.h"
#include "adns3080_srom.h"

static const char* LOG = "adns3080";

void initSpi(spi_device_handle_t* spiDevice, spi_host_device_t spiHost, gpio_num_t sclk,
        gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t spiConfig;
    spiConfig.mosi_io_num = mosi;
    spiConfig.miso_io_num = miso;
    spiConfig.sclk_io_num = sclk;
    spiConfig.quadwp_io_num = -1;
    spiConfig.quadhd_io_num = -1;
    spiConfig.max_transfer_sz = 0;
    spiConfig.flags = SPICOMMON_BUSFLAG_MASTER;
    spiConfig.intr_flags = 0;
    spi_bus_initialize(spiHost, &spiConfig, 0);

    spi_device_interface_config_t spiInterfaceConfig;
    spiInterfaceConfig.command_bits = 0;
    spiInterfaceConfig.address_bits = 0;
    spiInterfaceConfig.dummy_bits = 0;
    spiInterfaceConfig.mode = 3;
    spiInterfaceConfig.duty_cycle_pos = 128; // 50% duty cycle
    spiInterfaceConfig.cs_ena_pretrans = 0;
    spiInterfaceConfig.cs_ena_posttrans = 0;
    spiInterfaceConfig.clock_speed_hz = 2 * 1000 * 1000; // 2 MHz is max for ADNS-3080
    spiInterfaceConfig.input_delay_ns = 0; // FIXME 120;
    spiInterfaceConfig.spics_io_num = -1;
    spiInterfaceConfig.flags = SPI_DEVICE_HALFDUPLEX;
    spiInterfaceConfig.queue_size = 1;
    spiInterfaceConfig.pre_cb = NULL;
    spiInterfaceConfig.post_cb = NULL;

    spi_bus_add_device(spiHost, &spiInterfaceConfig, spiDevice);
}

ADNS3080::ADNS3080(spi_host_device_t spiHost, gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso,
        gpio_num_t cs) : cs(cs) {
    // note that we will be handling (N)CS ourselves
    gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    deassertCS();
    initSpi(&spiDevice, spiHost, sclk, mosi, miso);

    // for some reason the first register read is not reliable, so discard it
    ets_delay_us(50);
    readRegister(0x00);

    ets_delay_us(50);
    uint8_t productId = readRegister(0x00);
    ESP_LOGI(LOG, "product ID is %#.2x", productId);

    ets_delay_us(50);
    uint8_t configBits = readRegister(0x0A);
    ESP_LOGI(LOG, "configuration is %#.2x", configBits);

    configBits |= 0x10; // 1600 counts per inch
    writeRegister(0x0A, configBits);

    ets_delay_us(50);
    configBits = readRegister(0x0A);
    ESP_LOGI(LOG, "updated configuration is %#.2x", configBits);

    uploadSrom();
}

void ADNS3080::assertCS() {
    // CS is active low
    gpio_set_level(cs, 0);
}

void ADNS3080::deassertCS() {
    // CS is active low
    gpio_set_level(cs, 1);
}

void ADNS3080::writeRegister(uint8_t address, uint8_t data) {
    assertCS();
    ets_delay_us(1);
    
    spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.cmd = 0;
    tx.addr = 0;
    tx.length = 16;
    tx.rxlength = 0;
    tx.user = NULL;
    tx.tx_buffer = NULL;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0;
    tx.tx_data[0] = address | 0x80;
    tx.tx_data[1] = data;
    tx.rx_buffer = NULL;
    tx.rx_data[0] = tx.rx_data[1] = tx.rx_data[2] = tx.rx_data[3] = 0;

    spi_device_transmit(spiDevice, &tx);

    deassertCS();
}

void ADNS3080::writeRegisterBurst(uint8_t address, const uint8_t* data, size_t size) {
    assertCS();
    ets_delay_us(1);

    spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.cmd = 0;
    tx.addr = 0;
    tx.length = 8;
    tx.rxlength = 0;
    tx.user = NULL;
    tx.tx_buffer = NULL;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0;
    tx.tx_data[0] = address | 0x80;
    tx.rx_buffer = NULL;
    tx.rx_data[0] = tx.rx_data[1] = tx.rx_data[2] = tx.rx_data[3] = 0;
    spi_device_transmit(spiDevice, &tx);

    tx.length = 8;
    tx.rxlength = 8;
    for (int i = 0; i < size; i++) {
        tx.tx_data[0] = data[i];

        spi_device_transmit(spiDevice, &tx);
        ets_delay_us(10);
    }

    deassertCS();
}

uint8_t ADNS3080::readRegister(uint8_t address) {
    assertCS();
    ets_delay_us(1);

    spi_transaction_t tx;
    tx.flags = SPI_TRANS_USE_TXDATA;
    tx.cmd = 0;
    tx.addr = 0;
    tx.length = 8;
    tx.rxlength = 0;
    tx.user = NULL;
    tx.tx_buffer = NULL;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0;
    tx.tx_data[0] = address;
    tx.rx_buffer = NULL;
    tx.rx_data[0] = tx.rx_data[1] = tx.rx_data[2] = tx.rx_data[3] = 0;

    spi_device_transmit(spiDevice, &tx);
    if (address == 0x02) {
        ets_delay_us(75);
    } else {
        ets_delay_us(50);
    }

    tx.flags = SPI_TRANS_USE_RXDATA;
    tx.cmd = 0;
    tx.addr = 0;
    tx.length = 0;
    tx.rxlength = 8;
    tx.user = NULL;
    tx.tx_buffer = NULL;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0;
    tx.rx_buffer = NULL;
    tx.rx_data[0] = tx.rx_data[1] = tx.rx_data[2] = tx.rx_data[3] = 0;

    spi_device_transmit(spiDevice, &tx);

    deassertCS();

    return tx.rx_data[0];
}

void ADNS3080::uploadSrom() {
    // twiddle the magic bits
    ets_delay_us(50);
    writeRegister(0x20, 0x44);
    ets_delay_us(50);
    writeRegister(0x23, 0x07);
    ets_delay_us(50);
    writeRegister(0x24, 0x88);
    vTaskDelay(10 / portTICK_PERIOD_MS); // FIXME "at least one frame period"
    writeRegister(0x14, 0x18);
    ets_delay_us(50);

    int sromSize = sizeof(srom_bits);
    ESP_LOGI(LOG, "writing %d bytes to SROM", sromSize);
    writeRegisterBurst(0x60, srom_bits, sromSize);

    // raise CS to exit burst mode
    deassertCS();
    ets_delay_us(4);
    assertCS();

    ets_delay_us(50);
    uint8_t sromId = readRegister(0x1F);
    if (sromId == 0) {
        ESP_LOGW(LOG, "SROM upload failed");
    } else {
        ESP_LOGI(LOG, "running SRM code; SROM ID is %x", sromId);
    }
}

void ADNS3080::getMotion() {
    uint8_t motionBits = readRegister(0x02);
    if (!(motionBits & 0x80)) {
        // no motion has occurred
        return;
    }

    if (motionBits & 0x10) {
        // overflow has occurred
        ESP_LOGW(LOG, "motion overflow");
        return;
    }

    ets_delay_us(50);
    int8_t deltaX = readRegister(0x03);
    ets_delay_us(50);
    int8_t deltaY = readRegister(0x04);
    ets_delay_us(50);
    uint8_t sQual = readRegister(0x05);
    ESP_LOGI(LOG, "motion=%4d, %4d sQual=%3u", deltaX, deltaY, sQual);
}

char grayToAscii(uint8_t grayValue) {
    // only lower 6 bits are significant; "normalize" to the more familiar 8-bit grayscale
    grayValue <<= 2;

    const char* grayRamp = " .,:o@";
    int rampLength = strlen(grayRamp);

    return grayRamp[grayValue / (255 / rampLength)];
}

void ADNS3080::frameTest() {
    while (true) {
        ets_delay_us(50);
        writeRegister(0x13, 0x83);

        vTaskDelay(100 / portTICK_PERIOD_MS); // FIXME we probably don't need this long

        uint8_t frameBytes[900];
        memset(frameBytes, 0, 900);

        for (int i = 0; i < 900; i++) {
            ets_delay_us(50);
            frameBytes[i] = readRegister(0x13);
        }

        // frame data is right-to-left, bottom-to-top when looking "down"
        for (int i = 29; i >= 0; i--) {
            char scanline[31];
            scanline[30] = '\0';
            for (int j = 0; j < 30; j++) {
                scanline[j] = grayToAscii(*(frameBytes + (29 - j) + i * 30));
            }
            ESP_LOGI(LOG, "%s", scanline);
        }
    }
}

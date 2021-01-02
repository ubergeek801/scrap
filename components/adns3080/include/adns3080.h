#ifndef __ADNS3080_H__
#define __ADNS3080_H__

#include <stdint.h>

#include <driver/spi_master.h>

class ADNS3080 {
public:
    ADNS3080(spi_host_device_t spiHost, gpio_num_t sclk, gpio_num_t mosi, gpio_num_t miso,
            gpio_num_t cs);

    void getMotion();

    void frameTest();

protected:
    void assertCS();

    void deassertCS();

    void writeRegister(uint8_t address, uint8_t data);

    void writeRegisterBurst(uint8_t address, const uint8_t* data, size_t size);

    uint8_t readRegister(uint8_t address);

private:
    gpio_num_t cs;
    spi_device_handle_t spiDevice;
    
    void uploadSrom();
};

#endif

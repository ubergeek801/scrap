#ifndef __SCRAP_PLACARD_H__
#define __SCRAP_PLACARD_H__

#include <driver/gpio.h>
#include <driver/spi_master.h>

class ScrapPlacard {
public:
    ScrapPlacard(spi_host_device_t spiHost, gpio_num_t sclk, gpio_num_t mosi, gpio_num_t dc,
            gpio_num_t busy);

    void test();

private:
    gpio_num_t dc;
    gpio_num_t busy;
    spi_device_handle_t spiDevice;
};

#endif

#ifndef __PROPSHAFT_SENSOR_H__
#define __PROPSHAFT_SENSOR_H__

#include <driver/gpio.h>
#include <driver/pcnt.h>

class PropshaftSensor {
public:
    PropshaftSensor(gpio_num_t pin, pcnt_unit_t pcntUnit);

    uint16_t getPulseCount();

private:
    gpio_num_t pin;
    pcnt_unit_t pcntUnit;
};

#endif

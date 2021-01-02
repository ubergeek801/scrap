#include "propshaft_sensor.h"

PropshaftSensor::PropshaftSensor(gpio_num_t pin, pcnt_unit_t pcntUnit) : pin(pin),
        pcntUnit(pcntUnit) {
    pcnt_config_t pcntConfig;
    pcntConfig.pulse_gpio_num = pin;
    pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    pcntConfig.channel = PCNT_CHANNEL_0;
    pcntConfig.unit = pcntUnit;
    pcntConfig.pos_mode = PCNT_COUNT_INC;
    pcntConfig.neg_mode = PCNT_COUNT_INC;
    pcntConfig.lctrl_mode = PCNT_MODE_KEEP;
    pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
    pcntConfig.counter_l_lim = 0;
    pcntConfig.counter_h_lim = 0;
    pcnt_unit_config(&pcntConfig);

    // The pulse counter can treat short pulses as noise and filter them out; the value is in
    // APB_CLK ticks (typically 80 MHz). The max permitted value (1023) is still well below the
    // period of the propshaft signal, so just use the max.
    pcnt_set_filter_value(pcntUnit, 1023);
    pcnt_filter_enable(pcntUnit);

    // reset the counter
    pcnt_counter_pause(pcntUnit);
    pcnt_counter_clear(pcntUnit);

    // start counting
    pcnt_counter_resume(pcntUnit);
}

uint16_t PropshaftSensor::getPulseCount() {
    int16_t count;
    pcnt_get_counter_value(pcntUnit, &count);
    pcnt_counter_clear(pcntUnit);

    return count;
}

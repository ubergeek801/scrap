#ifndef __PWM_GENERATOR_H__
#define __PWM_GENERATOR_H__

#include "mcpwm_device.h"

class PWMGenerator {
public:
    PWMGenerator(gpio_num_t outputPin, MCPWMDevice& mcpwmDevice, mcpwm_io_signals_t mcpwmSignal,
            mcpwm_timer_t mcpwmTimer, mcpwm_generator_t mcpwmGenerator);

    void setDutyCycle(uint32_t dutyInUs);

protected:

private:
    gpio_num_t outputPin;
    MCPWMDevice& mcpwmDevice;
    mcpwm_timer_t mcpwmTimer;
    mcpwm_generator_t mcpwmGenerator;
};

#endif

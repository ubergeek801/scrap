#include "pwm_generator.h"

// a pretty arbitrary choice
#define MCPWM_TIMER MCPWM_TIMER_0

PWMGenerator::PWMGenerator(gpio_num_t outputPin, MCPWMDevice& mcpwmDevice,
        mcpwm_io_signals_t mcpwmSignal, mcpwm_timer_t mcpwmTimer, mcpwm_generator_t mcpwmGenerator)
        : outputPin(outputPin), mcpwmDevice(mcpwmDevice), mcpwmTimer(mcpwmTimer),
        mcpwmGenerator(mcpwmGenerator) {
    mcpwm_gpio_init(mcpwmDevice.mcpwmUnit, mcpwmSignal, outputPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // 50Hz, i.e. 20ms period
    pwm_config.cmpr_a = 7.5; // default to "neutral" = 1.5ms / 20ms = 7.5% duty cycle
    pwm_config.cmpr_b = 7.5; // default to "neutral"
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
    mcpwm_init(mcpwmDevice.mcpwmUnit, mcpwmTimer, &pwm_config);
}

void PWMGenerator::setDutyCycle(uint32_t dutyInUs) {
    mcpwm_set_duty_in_us(mcpwmDevice.mcpwmUnit, mcpwmTimer, mcpwmGenerator, dutyInUs);
}

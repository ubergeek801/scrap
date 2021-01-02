#ifndef __RC_REPEATER_H__
#define __RC_REPEATER_H__

#include "pwm_generator.h"

class RCRepeater : public CaptureEventHandler {
public:
    RCRepeater(const char* name, gpio_num_t inputPin, MCPWMDevice& mcpwmDevice,
            const CaptureSignal& captureSignal, PWMGenerator& outputGenerator);

    void onEdge(uint32_t captureValue, bool edge);

protected:
    static void setPwmOutput(void* param);

    int8_t convertPulseToPosition(int16_t pulseWidth);

private:
    const char* name;
    gpio_num_t inputPin;
    MCPWMDevice& mcpwmDevice;
    const CaptureSignal& captureSignal;
    PWMGenerator& outputGenerator;

    uint32_t pulseBeginTimestamp = 0;
    int8_t position = 0;
    bool timeout = true;
    uint64_t positionTimestamp = 0;
};

#endif

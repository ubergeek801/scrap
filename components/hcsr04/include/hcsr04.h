#ifndef __HCSR04_H__
#define __HCSR04_H__

#include "mcpwm_device.h"

class HCSR04Trigger;

class HCSR04Receiver : public CaptureEventHandler {
public:
    void onEdge(uint32_t captureValue, bool edge);

protected:
    HCSR04Receiver(const gpio_num_t echoPin, MCPWMDevice& mcpwmDevice,
            const CaptureSignal& captureSignal);

    void setTrigger(const HCSR04Trigger* trigger);

    virtual void echo(uint16_t mmDistance, const HCSR04Trigger* trigger) = 0;

private:
    const gpio_num_t echoPin;
    MCPWMDevice& mcpwmDevice;
    const CaptureSignal& captureSignal;

    uint32_t pulseBeginTimestamp;
    const HCSR04Trigger* trigger;

    friend class HCSR04Trigger;
};

class HCSR04Trigger {
public:
    HCSR04Trigger(uint8_t id, HCSR04Receiver* echoReceiver, const gpio_num_t triggerPin);

    uint8_t getId() const { return id; }

    void ping();

private:
    const uint8_t id;
    HCSR04Receiver* echoReceiver;
    const gpio_num_t triggerPin;
};

#endif

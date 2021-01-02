#ifndef __MCPWM_DEVICE_H__
#define __MCPWM_DEVICE_H__

#include <stdint.h>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>

/**
 * An attempt at a more usable abstraction of the ESP32's PWM Capture configuration than what is
 * provided by the ESP-IDF API. A CaptureSignal encapsulates mcpwm_io_signals_t,
 * mcpwm_capture_signal_t and mcpwm_intr_t, and is used to register interest in a signal with a
 * MCPWMDevice.
 */
class CaptureSignal {
public:
    static CaptureSignal& captureSignal0() { static CaptureSignal signal0(MCPWM_CAP_0,
            MCPWM_SELECT_CAP0, MCPWM_LL_INTR_CAP0); return signal0; }
    static CaptureSignal& captureSignal1() { static CaptureSignal signal1(MCPWM_CAP_1,
            MCPWM_SELECT_CAP1, MCPWM_LL_INTR_CAP1); return signal1; }
    static CaptureSignal& captureSignal2() { static CaptureSignal signal2(MCPWM_CAP_2,
            MCPWM_SELECT_CAP2, MCPWM_LL_INTR_CAP2); return signal2; }

    bool operator==(const CaptureSignal& other) const { return mcpwmSignal == other.mcpwmSignal
            && mcpwmCapture == other.mcpwmCapture && mcpwmInterrupt == other.mcpwmInterrupt; }

protected:
    CaptureSignal(mcpwm_io_signals_t mcpwmSignal, mcpwm_capture_signal_t mcpwmCapture,
            mcpwm_intr_t mcpwmInterrupt) : mcpwmSignal(mcpwmSignal), mcpwmCapture(mcpwmCapture),
            mcpwmInterrupt(mcpwmInterrupt) { }

private:
    const mcpwm_io_signals_t mcpwmSignal;
    const mcpwm_capture_signal_t mcpwmCapture;
    const mcpwm_intr_t mcpwmInterrupt;

    friend class MCPWMDevice;
};

class CaptureEventHandler {
public:
    virtual void onEdge(uint32_t captureValue, bool edge) = 0;
};

struct CaptureSignalRegistration {
    const CaptureSignal& captureSignal;
    CaptureEventHandler* eventHandler;

    CaptureSignalRegistration(const CaptureSignal& captureSignal,
            CaptureEventHandler* eventHandler) : captureSignal(captureSignal),
            eventHandler(eventHandler) { }
};

/**
 * An attempt at a more usable abstraction of the ESP32's MCPWM devices than what is provided by
 * the ESP-IDF API. A MCPWMDevice encapsulates both mcpwm_dev_t and mcwpm_unit_t, and provides
 * facilities to manage capture interrupts for otherwise independent devices (since these must
 * share a single ISR per unit).
 */
class MCPWMDevice {
public:
    static MCPWMDevice& mcpwmDevice0() { static MCPWMDevice device0(MCPWM0, MCPWM_UNIT_0);
            return device0; }
    static MCPWMDevice& mcpwmDevice1() { static MCPWMDevice device1(MCPWM1, MCPWM_UNIT_1);
            return device1; }

    void registerSignal(const gpio_num_t pin, const CaptureSignal& signal,
            CaptureEventHandler* eventHandler);

    void unregisterSignal(const CaptureSignal& signal);

protected:
    MCPWMDevice(mcpwm_dev_t& mcpwm, mcpwm_unit_t mcpwmUnit);

    static void mcpwmCaptureIsrHandler(void* device);

    static void handlePwmCaptureEvents(void* device);

private:
    mcpwm_dev_t& mcpwm;
    const mcpwm_unit_t mcpwmUnit;

    xQueueHandle captureEventQueue;

    std::vector<const CaptureSignalRegistration*> signalRegistrations;

    friend class PWMGenerator;
};

#endif

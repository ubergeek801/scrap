#include "mcpwm_device.h"

#include <esp_log.h>

struct PwmCaptureEvent {
    const uint32_t captureValue;
    const uint8_t edge;
    const CaptureSignal* captureSignal;

    PwmCaptureEvent(const uint32_t captureValue, const uint8_t edge,
            const CaptureSignal* captureSignal) : captureValue(captureValue), edge(edge),
            captureSignal(captureSignal) { }
};

static const CaptureSignal* captureSignals[] = { &CaptureSignal::captureSignal0(),
        &CaptureSignal::captureSignal1(), &CaptureSignal::captureSignal2() };

static const char* LOG = "mcpwm_device";

void IRAM_ATTR MCPWMDevice::mcpwmCaptureIsrHandler(void* device) {
    const MCPWMDevice* mcpwmDevice = (const MCPWMDevice*)device;
    mcpwm_dev_t& mcpwm = mcpwmDevice->mcpwm;

    uint32_t pwmInterruptStatus = mcpwm.int_st.val;

    mcpwm_unit_t mcpwmUnit = mcpwmDevice->mcpwmUnit;
    for (int i = 0; i < 3; i++) {
        if (pwmInterruptStatus & captureSignals[i]->mcpwmInterrupt) {
            const mcpwm_capture_signal_t mcpwmCapture = captureSignals[i]->mcpwmCapture;

            uint32_t captureValue = mcpwm_capture_signal_get_value(mcpwmUnit, mcpwmCapture);
            uint8_t edge = mcpwm_capture_signal_get_edge(mcpwmUnit, mcpwmCapture);

            PwmCaptureEvent captureEvent(captureValue, edge, captureSignals[i]);

            xQueueHandle captureEventQueue = mcpwmDevice->captureEventQueue;
            xQueueSendFromISR(captureEventQueue, &captureEvent, NULL);
        }
    }

    // clear the interrupt
    mcpwm.int_clr.val = pwmInterruptStatus;
}

void MCPWMDevice::handlePwmCaptureEvents(void* device) {
    const MCPWMDevice* mcpwmDevice = (const MCPWMDevice*)device;
    xQueueHandle captureEventQueue = mcpwmDevice->captureEventQueue;

    PwmCaptureEvent captureEvent(0, 0, NULL); // values will be overwritten by received event
    while (true) {
        xQueueReceive(captureEventQueue, &captureEvent, portMAX_DELAY);

        // find the handler for this signal, if any
        CaptureEventHandler* eventHandler = NULL;
        for (const CaptureSignalRegistration* registration : mcpwmDevice->signalRegistrations) {
            if (*captureEvent.captureSignal == registration->captureSignal) {
                eventHandler = registration->eventHandler;
                break;
            }
        }

        if (eventHandler == NULL) {
            // odd, but ignore the event in this case
            ESP_LOGW(LOG, "no handler registered for interrupt %x",
                    captureEvent.captureSignal->mcpwmInterrupt);
            continue;
        }

        if (captureEvent.edge == 1 /* positive edge, opposite of mcpwm_capture_on_edge_t! */) {
            eventHandler->onEdge(captureEvent.captureValue, true);
        } else {
            eventHandler->onEdge(captureEvent.captureValue, false);
        }
    }
}

MCPWMDevice::MCPWMDevice(mcpwm_dev_t& mcpwm, mcpwm_unit_t mcpwmUnit) : mcpwm(mcpwm),
        mcpwmUnit(mcpwmUnit), signalRegistrations() {
    captureEventQueue = xQueueCreate(10, sizeof(PwmCaptureEvent));

    xTaskCreate(MCPWMDevice::handlePwmCaptureEvents, "handlePwmCapture" + (int)mcpwmUnit, 4096,
            this, 5, NULL);

    ESP_ERROR_CHECK(mcpwm_isr_register(mcpwmUnit, MCPWMDevice::mcpwmCaptureIsrHandler, this,
            ESP_INTR_FLAG_IRAM, NULL));
}

void MCPWMDevice::registerSignal(const gpio_num_t pin, const CaptureSignal& signal,
        CaptureEventHandler* eventHandler) {
    gpio_pulldown_en(pin);

    ESP_ERROR_CHECK(mcpwm_gpio_init(mcpwmUnit, signal.mcpwmSignal, pin));
    mcpwm.int_ena.val |= signal.mcpwmInterrupt;

    CaptureSignalRegistration* registration = new CaptureSignalRegistration(signal, eventHandler);
    signalRegistrations.push_back(registration);

    ESP_ERROR_CHECK(mcpwm_capture_enable(mcpwmUnit, signal.mcpwmCapture, MCPWM_BOTH_EDGE, 0));
}

void MCPWMDevice::unregisterSignal(const CaptureSignal& signal) {
    for (auto registrationIter = signalRegistrations.begin(); registrationIter
            != signalRegistrations.end(); registrationIter++) {
        const CaptureSignalRegistration* registration = *registrationIter;
        if (signal == registration->captureSignal) {
            signalRegistrations.erase(registrationIter);
            ESP_ERROR_CHECK(mcpwm_capture_disable(mcpwmUnit, signal.mcpwmCapture));
            mcpwm.int_ena.val &= ~signal.mcpwmInterrupt;
            delete registration;
            // assume there is at most one such registration
            break;
        }
    }
}

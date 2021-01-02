#include "hcsr04.h"

#include <soc/rtc.h>
#include <esp_log.h>

static const char* LOG = "hcsr04";

HCSR04Receiver::HCSR04Receiver(const gpio_num_t echoPin, MCPWMDevice& mcpwmDevice,
        const CaptureSignal& captureSignal) : echoPin(echoPin), mcpwmDevice(mcpwmDevice),
        captureSignal(captureSignal) {
    mcpwmDevice.registerSignal(echoPin, captureSignal, this);
}

void HCSR04Receiver::onEdge(uint32_t captureValue, bool edge) {
    if (edge) {
        // this is a positive edge; capture the start time
        pulseBeginTimestamp = captureValue;
    } else {
        // this is a negative edge; generate an echo event
        uint32_t duration = captureValue - pulseBeginTimestamp;
        uint32_t durationUs = duration / (rtc_clk_apb_freq_get() / 1000000);
        // convert microseconds to mm using the speed of sound and account for a round trip
        uint16_t mmDistance = (durationUs * 343 / 1000) / 2;

        echo(mmDistance, trigger);
    }
}

void HCSR04Receiver::setTrigger(const HCSR04Trigger* trigger) {
    this->trigger = trigger;
}

HCSR04Trigger::HCSR04Trigger(uint8_t id, HCSR04Receiver* echoReceiver, const gpio_num_t triggerPin)
        : id(id), echoReceiver(echoReceiver), triggerPin(triggerPin) {
    gpio_pad_select_gpio(triggerPin);
    gpio_set_direction(triggerPin, GPIO_MODE_OUTPUT);
    gpio_set_level(triggerPin, 0);
}

void HCSR04Trigger::ping() {
    // let the Receiver know that something is coming
    echoReceiver->setTrigger(this);
    
    gpio_set_level(triggerPin, 1);
    ets_delay_us(10);
    gpio_set_level(triggerPin, 0);

    // PWM capture and HCSR04Receiver will handle the rest
}

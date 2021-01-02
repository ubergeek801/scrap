#include "rc_repeater.h"

#include <freertos/FreeRTOS.h>
#include <soc/rtc.h>
#include <esp_log.h>

// characteristics of PWM used for R/C: 1000us <= pulse width <= 2000us (1500us is "neutral")
#define PULSE_TOLERANCE_US 100 // a pulse this far out of the expected range is ignored
#define MIN_PULSE_US 1000 // pulse width corresponding to the "low" limit
#define MAX_PULSE_US 2000 // pulse widh corresponding to the "high" limit
#define NEUTRAL_PULSE_MS ((MIN_PULSE_US + MAX_PULSE_US) >> 1) // "neutral" pulse width
#define HALF_PULSE_US (MAX_PULSE_US - NEUTRAL_PULSE_MS) // difference between neutral and a limit
// no valid pulses for this period of time forces neutral output
#define SIGNAL_TIMEOUT_US (250 * 1000)

#define PWM_UPDATE_MS 20 // interval between throttle/steering PWM output updates

static const char* LOG = "rc_repeater";

RCRepeater::RCRepeater(const char* name, gpio_num_t inputPin, MCPWMDevice& mcpwmDevice,
        const CaptureSignal& captureSignal, PWMGenerator& outputGenerator) : name(name),
        inputPin(inputPin), mcpwmDevice(mcpwmDevice), captureSignal(captureSignal),
        outputGenerator(outputGenerator) {
    mcpwmDevice.registerSignal(inputPin, captureSignal, this);

    xTaskCreate(setPwmOutput, "setPwmOutput", 4096, this, 5, NULL);
}

void RCRepeater::onEdge(uint32_t captureValue, bool edge) {
    if (edge) {
        // this is a positive edge; capture the start time
        pulseBeginTimestamp = captureValue;
    } else {
        // this is a negative edge; record the pulse measurement, converting from ticks to us
        uint32_t duration = (captureValue - pulseBeginTimestamp) / (rtc_clk_apb_freq_get()
                / 1000000);
        if (duration < MIN_PULSE_US - PULSE_TOLERANCE_US || duration > MAX_PULSE_US +
                PULSE_TOLERANCE_US) {
            // period is way outside expected range, so treat it as a positive edge that wasn't
            // detected properly
            pulseBeginTimestamp = captureValue;
        } else {
            position = convertPulseToPosition(duration);
            positionTimestamp = esp_timer_get_time();
            timeout = false;
        }
    }
}

int8_t RCRepeater::convertPulseToPosition(int16_t pulseWidth) {
    // pulseWidth should theoretically be between 1000 and 2000 microseconds (1500 +/- 500), which
    // we rescale to int8_t range (0 +/- 127)

    int32_t scaledPulseWidth = INT8_MAX * (pulseWidth - NEUTRAL_PULSE_MS) / HALF_PULSE_US;

    return (int8_t)(scaledPulseWidth < INT8_MIN ? INT8_MIN :
            scaledPulseWidth > INT8_MAX ? INT8_MAX : scaledPulseWidth);
}

void RCRepeater::setPwmOutput(void* param) {
    RCRepeater* repeater = (RCRepeater*)param;

    while (true) {
        uint64_t now = esp_timer_get_time();
        if (now - repeater->positionTimestamp > SIGNAL_TIMEOUT_US) {
            repeater->position = 0;
            repeater->positionTimestamp = now;
            if (!repeater->timeout) {
                ESP_LOGW(LOG, "timeout on R/C input channel \"%s\"", repeater->name);
            }
            repeater->timeout = true;
        }

        uint32_t period = (repeater->position * HALF_PULSE_US) / INT8_MAX + NEUTRAL_PULSE_MS;
        repeater->outputGenerator.setDutyCycle(period);

        vTaskDelay(PWM_UPDATE_MS / portTICK_PERIOD_MS);
    }
}

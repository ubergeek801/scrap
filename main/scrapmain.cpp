#include <freertos/FreeRTOS.h>
#include <driver/adc.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>
#include <soc/rtc.h>
#include <esp_adc_cal.h>

#include "scraplogo.h"
#include "sdkconfig.h"
#include "esp32_u8g2/u8g2_esp32.h"

// interrupt enable bits for MCPWM capture, but not defined in mcpwm.h
#define PWM_CAP0_INT_ENABLE_BIT BIT(27)
#define PWM_CAP1_INT_ENABLE_BIT BIT(28)
#define PWM_CAP2_INT_ENABLE_BIT BIT(29)

// these conventions are alluded to but not defined in mcpwm.h
#define PWM_RISING_EDGE BIT(0)
#define PWM_FALLING_EDGE BIT(1)

// parameters for INA169 DC current sensor
#define INA169_ADC1_CHANNEL ADC1_CHANNEL_6 // GPIO 34
#define INA169_ADC_CHANNEL ADC_CHANNEL_6

// parameters for SSD1306 communication via I2C
#define DISPLAY_I2C_SCL_PIN ((gpio_num_t)22)
#define DISPLAY_I2C_SDA_PIN ((gpio_num_t)21)
#define DISPLAY_I2C_PORT I2C_NUM_1
#define DISPLAY_I2C_ADDRESS (0x3C << 1) // address is actually 0x3C but API expects << 1
#define DISPLAY_I2C_CLOCK_SPEED ((uint32_t)400000)

#define FXOS8700CQ_I2C_ADDRESS (0x1E << 1) // address is actually 0x1E but API expects << 1
#define FXAS21002CQ_I2C_ADDRESS (0x20 << 1) // address is actually 0x20 but API expects << 1

// MCPWM configuration for throttle and steering input signals
#define THROTTLE_STEERING_MCPWM MCPWM0
#define THROTTLE_STEERING_MCPWM_UNIT MCPWM_UNIT_0
#define THROTTLE_STEERING_MCPWM_TIMER MCPWM_TIMER_0

// throttle input MCPWM configuration
#define THROTTLE_INPUT_PIN ((gpio_num_t)25)
#define THROTTLE_MCPWM_CAP MCPWM_CAP_0
#define THROTTLE_MCPWM_SELECT MCPWM_SELECT_CAP0
#define THROTTLE_INT_ENABLE_BIT PWM_CAP0_INT_ENABLE_BIT

// steering input MCPWM configuration
#define STEERING_INPUT_PIN ((gpio_num_t)26)
#define STEERING_MCPWM_CAP MCPWM_CAP_1
#define STEERING_MCPWM_SELECT MCPWM_SELECT_CAP1
#define STEERING_INT_ENABLE_BIT PWM_CAP1_INT_ENABLE_BIT

// throttle output MCPWM configuration
#define THROTTLE_OUTPUT_PIN ((gpio_num_t)4)
#define THROTTLE_OUTPUT_SIGNAL MCPWM0A
#define THROTTLE_OUTPUT_OPERATOR MCPWM_OPR_A

// steering output MCPWM configuration
#define STEERING_OUTPUT_PIN ((gpio_num_t)13)
#define STEERING_OUTPUT_SIGNAL MCPWM0B
#define STEERING_OUTPUT_OPERATOR MCPWM_OPR_B

// characteristics of PWM used for R/C: 1000us <= pulse width <= 2000us (1500us is "neutral")
#define PULSE_TOLERANCE_US 100 // a pulse this far out of the expected range is ignored
#define MIN_PULSE_US 1000 // pulse width corresponding to the "low" limit
#define MAX_PULSE_US 2000 // pulse widh corresponding to the "high" limit
#define NEUTRAL_PULSE_MS ((MIN_PULSE_US + MAX_PULSE_US) >> 1) // "neutral" pulse width
#define HALF_PULSE_US (MAX_PULSE_US - NEUTRAL_PULSE_MS) // difference between neutral and a limit
#define SIGNAL_TIMEOUT_US 1000000 // no valid pulses for this period of time forces neutral output

// miscellaneous delays
#define SPLASH_SCREEN_DELAY_MS 8000 // display the splash screen for this period
#define PWM_UPDATE_MS 100 // interval between throttle/steering PWM output updates
#define DISPLAY_UPDATE_MS 100 // interval between display updates

typedef enum {
	SOURCE_THROTTLE = 0,
	SOURCE_STEERING,
	NUM_PWM_SOURCES
} pwm_event_source_t;

typedef struct pwm_event_s {
	pwm_event_source_t source;
	uint32_t captureValue;
	uint8_t edge;
} pwm_event_t;

typedef struct pwm_event_context_s {
	const char* name;
	volatile uint32_t previousValue;
	volatile uint32_t currentValue;
	void (*handler)(int8_t);
} pwm_event_context_t;

SemaphoreHandle_t printMux = NULL;

U8G2* scrapDisplay = NULL;

xQueueHandle pwmEventQueue;

esp_adc_cal_characteristics_t adc1Characteristics;

// FIXME maybe these should be atomic
static volatile int8_t throttlePosition = 0;
static volatile uint64_t throttlePositionTimestamp = 0;
static volatile int8_t steeringPosition = 0;
static volatile uint64_t steeringPositionTimestamp = 0;

pwm_event_context_t pwmEventContexts[NUM_PWM_SOURCES];

int8_t convertPulseToPosition(int16_t pulseWidth) {
	// pulseWidth should theoretically be between 1000 and 2000 microseconds (1500 +/- 500), which
	// we rescale to int8_t range (0 +/- 127)

	int32_t scaledPulseWidth = INT8_MAX * (pulseWidth - NEUTRAL_PULSE_MS) / HALF_PULSE_US;

	return (int8_t) (scaledPulseWidth < INT8_MIN ? INT8_MIN :
			scaledPulseWidth > INT8_MAX ? INT8_MAX : scaledPulseWidth);
}

void processPwm(void* arg) {
	pwm_event_t pwmEvent;

	while (true) {
		xQueueReceive(pwmEventQueue, &pwmEvent, portMAX_DELAY);
		pwm_event_context_t* eventContext = pwmEventContexts + pwmEvent.source;
		eventContext->previousValue = eventContext->currentValue;
		eventContext->currentValue = pwmEvent.captureValue;
		uint32_t duration = (eventContext->currentValue - eventContext->previousValue)
				/ (rtc_clk_apb_freq_get() / 1000000);
		if (pwmEvent.edge == PWM_FALLING_EDGE) {
			if (duration < MIN_PULSE_US - PULSE_TOLERANCE_US || duration > MAX_PULSE_US +
					PULSE_TOLERANCE_US) {
				// period is way outside expected range, so discard this pulse
			} else {
				eventContext->handler(convertPulseToPosition(duration));
			}
		}
	}
}

void IRAM_ATTR mcpwmIsrHandler(void* param) {
	pwm_event_t pwmEvent;
	uint32_t mcpwm_intr_status = THROTTLE_STEERING_MCPWM.int_st.val; //Read interrupt status
	if (mcpwm_intr_status & THROTTLE_INT_ENABLE_BIT) {
		pwmEvent.source = SOURCE_THROTTLE;
		pwmEvent.captureValue = mcpwm_capture_signal_get_value(THROTTLE_STEERING_MCPWM_UNIT,
				THROTTLE_MCPWM_SELECT);
		pwmEvent.edge = mcpwm_capture_signal_get_edge(THROTTLE_STEERING_MCPWM_UNIT,
				THROTTLE_MCPWM_SELECT);
		xQueueSendFromISR(pwmEventQueue, &pwmEvent, NULL);
	}
	if (mcpwm_intr_status & STEERING_INT_ENABLE_BIT) {
		pwmEvent.source = SOURCE_STEERING;
		pwmEvent.captureValue = mcpwm_capture_signal_get_value(THROTTLE_STEERING_MCPWM_UNIT,
				STEERING_MCPWM_SELECT);
		pwmEvent.edge = mcpwm_capture_signal_get_edge(THROTTLE_STEERING_MCPWM_UNIT,
				STEERING_MCPWM_SELECT);
		xQueueSendFromISR(pwmEventQueue, &pwmEvent, NULL);
	}
	THROTTLE_STEERING_MCPWM.int_clr.val = mcpwm_intr_status;
}

void initGpio() {
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(INA169_ADC1_CHANNEL, ADC_ATTEN_DB_0);
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100,
			&adc1Characteristics);

	mcpwm_gpio_init(THROTTLE_STEERING_MCPWM_UNIT, THROTTLE_MCPWM_CAP, THROTTLE_INPUT_PIN);
	mcpwm_gpio_init(THROTTLE_STEERING_MCPWM_UNIT, STEERING_MCPWM_CAP, STEERING_INPUT_PIN);
    mcpwm_gpio_init(THROTTLE_STEERING_MCPWM_UNIT, THROTTLE_OUTPUT_SIGNAL, THROTTLE_OUTPUT_PIN);
    mcpwm_gpio_init(THROTTLE_STEERING_MCPWM_UNIT, STEERING_OUTPUT_SIGNAL, STEERING_OUTPUT_PIN);

	gpio_pulldown_en(THROTTLE_INPUT_PIN);
	gpio_pulldown_en(STEERING_INPUT_PIN);
}

void initMcpwm() {
    mcpwm_config_t pwm_config;
	pwm_config.frequency = 50; // 50Hz, i.e. 20ms period
	pwm_config.cmpr_a = 7.5; // default to "neutral" = 1.5ms / 20ms = 7.5% duty cycle
	pwm_config.cmpr_b = 7.5; // default to "neutral"
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
    mcpwm_init(THROTTLE_STEERING_MCPWM_UNIT, THROTTLE_STEERING_MCPWM_TIMER, &pwm_config);

    // the ability to capture both rising and falling edges relies on a hack to mcpwm.c, at least
	// until this IDF issue is addressed: https://github.com/espressif/esp-idf/issues/1907
	mcpwm_capture_enable(THROTTLE_STEERING_MCPWM_UNIT, THROTTLE_MCPWM_SELECT,
			(mcpwm_capture_on_edge_t)(PWM_RISING_EDGE | PWM_FALLING_EDGE), 0);
	mcpwm_capture_enable(THROTTLE_STEERING_MCPWM_UNIT, STEERING_MCPWM_SELECT,
			(mcpwm_capture_on_edge_t)(PWM_RISING_EDGE | PWM_FALLING_EDGE), 0);
	THROTTLE_STEERING_MCPWM.int_ena.val = THROTTLE_INT_ENABLE_BIT | STEERING_INT_ENABLE_BIT;
}

void throttleHandler(int8_t newThrottlePosition) {
	throttlePosition = newThrottlePosition;
	throttlePositionTimestamp = esp_timer_get_time();
}

void steeringHandler(int8_t newSteeringPosition) {
	steeringPosition = newSteeringPosition;
	steeringPositionTimestamp = esp_timer_get_time();
}

void setPwmOutput(void* param) {
	while (1) {
		uint64_t now = esp_timer_get_time();
		if (now - throttlePositionTimestamp > SIGNAL_TIMEOUT_US) {
			throttlePosition = 0;
			throttlePositionTimestamp = now;
		}
		if (now - steeringPositionTimestamp > SIGNAL_TIMEOUT_US) {
			steeringPosition = 0;
			throttlePositionTimestamp = now;
		}

		uint32_t period = (throttlePosition * HALF_PULSE_US) / INT8_MAX + NEUTRAL_PULSE_MS;
		mcpwm_set_duty_in_us(THROTTLE_STEERING_MCPWM_UNIT, THROTTLE_STEERING_MCPWM_TIMER,
				THROTTLE_OUTPUT_OPERATOR, period);
		period = (steeringPosition * HALF_PULSE_US) / INT8_MAX + NEUTRAL_PULSE_MS;
		mcpwm_set_duty_in_us(THROTTLE_STEERING_MCPWM_UNIT, THROTTLE_STEERING_MCPWM_TIMER,
				STEERING_OUTPUT_OPERATOR, period);

		vTaskDelay(PWM_UPDATE_MS / portTICK_PERIOD_MS);
	}
}

void updateDisplay(void* param) {
	while (true) {
		vTaskDelay(DISPLAY_UPDATE_MS / portTICK_PERIOD_MS);

		char messageBuffer[80];
		scrapDisplay->setDrawColor(0);
		scrapDisplay->drawBox(36, 12, 4 * 4, 2 * 6);
		scrapDisplay->setFont(u8g2_font_tom_thumb_4x6_tr);
		scrapDisplay->setDrawColor(1);

		sprintf(messageBuffer, "steering %d", steeringPosition);
		scrapDisplay->drawStr(0, 12, messageBuffer);

		sprintf(messageBuffer, "throttle %d", throttlePosition);
		scrapDisplay->drawStr(0, 18, messageBuffer);

		// INA169 output is wired such that 1 mV = 1 mA of current
		uint32_t ina169mV;
		esp_adc_cal_get_voltage(INA169_ADC_CHANNEL, &adc1Characteristics, &ina169mV);
		sprintf(messageBuffer, "current  %d mA", ina169mV);
		scrapDisplay->drawStr(0, 24, messageBuffer);

		scrapDisplay->sendBuffer();
	}
}

extern "C" void app_main() {
	printMux = xSemaphoreCreateMutex();

	pwmEventContexts[0].name = "throttle";
	pwmEventContexts[0].previousValue = 0;
	pwmEventContexts[0].currentValue = 0;
	pwmEventContexts[0].handler = throttleHandler;

	pwmEventContexts[1].name = "steering";
	pwmEventContexts[1].previousValue = 0;
	pwmEventContexts[1].currentValue = 0;
	pwmEventContexts[1].handler = steeringHandler;

	initGpio();
	initMcpwm();

	scrapDisplay = new U8G2_SSD1306_128X32_GENERIC_I2C(DISPLAY_I2C_PORT, DISPLAY_I2C_ADDRESS,
			DISPLAY_I2C_CLOCK_SPEED, DISPLAY_I2C_SDA_PIN, DISPLAY_I2C_SCL_PIN);
	scrapDisplay->begin();
	scrapDisplay->setFlipMode(true);

	pwmEventQueue = xQueueCreate(5, sizeof(pwm_event_t));
	xTaskCreate(processPwm, "processPwm", 4096, NULL, 5, NULL);
	xTaskCreate(setPwmOutput, "setPwmOutput", 4096, NULL, 5, NULL);
	mcpwm_isr_register(THROTTLE_STEERING_MCPWM_UNIT, mcpwmIsrHandler, NULL, ESP_INTR_FLAG_IRAM,
			NULL);

	scrapDisplay->drawXBM(0, 0, 128, 32, scrap_bits);
	scrapDisplay->sendBuffer();

	vTaskDelay(SPLASH_SCREEN_DELAY_MS / portTICK_PERIOD_MS);

	scrapDisplay->clearBuffer();
	scrapDisplay->setFont(u8g2_font_tom_thumb_4x6_tr);
	scrapDisplay->setFontPosTop();
	scrapDisplay->drawStr(0, 0, "Scale Car Racing");
	scrapDisplay->drawStr(0, 6, "Automation Platform");
	scrapDisplay->drawStr(0, 12, "steering");
	scrapDisplay->drawStr(0, 18, "throttle");
	scrapDisplay->sendBuffer();
	xTaskCreate(updateDisplay, "updateDisplay", 4096, NULL, 5, NULL);
}

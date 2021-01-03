#include <cstring>

#include <freertos/FreeRTOS.h>
#include <driver/adc.h>
#include <soc/adc_channel.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include "sdkconfig.h"

#include "adns3080.h"
#include "hcsr04.h"
#include "nxp_imu.h"
#include "propshaft_sensor.h"
#include "pwm_generator.h"
#include "rc_repeater.h"
#include "scrap_display.h"
#include "scrap_placard.h"
#include "telemetry.h"
#include "wifi.h"

#define WIFI_SSID "SCRAP"
#define WIFI_PASSWORD "5cr4ppyN37w0rk"

// parameters for current draw sensor, battery voltage sensor, UI button resistor ladder
#define SYS_CURRENT_ADC1_CHANNEL ADC1_GPIO36_CHANNEL
#define SYS_CURRENT_ADC_CHANNEL ADC_CHANNEL_0 // GPIO 36
#define SYS_VOLTAGE_ADC1_CHANNEL ADC1_GPIO39_CHANNEL
#define SYS_VOLTAGE_ADC_CHANNEL ADC_CHANNEL_3 // GPIO 39
#define UI_BUTTONS_ADC1_CHANNEL ADC1_GPIO38_CHANNEL
#define UI_BUTTONS_ADC_CHANNEL ADC_CHANNEL_2 // GPIO 38

#define PERIPHERAL_RESET_PIN GPIO_NUM_2

#define SONAR_RIGHT_PING_PIN GPIO_NUM_12
#define SONAR_FRONT_PING_PIN GPIO_NUM_9
#define SONAR_LEFT_PING_PIN GPIO_NUM_10
#define SONAR_ECHO_PIN GPIO_NUM_34
static MCPWMDevice& sonarMcpwmDevice = MCPWMDevice::mcpwmDevice1();
static CaptureSignal& sonarCaptureSignal = CaptureSignal::captureSignal0();

// parameters for OLED communication via I2C
#define DISPLAY_I2C_PORT I2C_NUM_1
#define DISPLAY_I2C_SCL_PIN GPIO_NUM_5
#define DISPLAY_I2C_SDA_PIN GPIO_NUM_18
#define DISPLAY_I2C_ADDRESS 0x3C

#define IMU_I2C_PORT I2C_NUM_0
#define IMU_I2C_SCL_PIN GPIO_NUM_15
#define IMU_I2C_SDA_PIN GPIO_NUM_4

// MCPWM configuration for throttle and steering input signals
static MCPWMDevice& rcMcpwmDevice = MCPWMDevice::mcpwmDevice0();
static CaptureSignal& throttleCaptureSignal = CaptureSignal::captureSignal0();
static CaptureSignal& steeringCaptureSignal = CaptureSignal::captureSignal1();

// throttle input MCPWM configuration
#define THROTTLE_INPUT_PIN GPIO_NUM_21

// steering input MCPWM configuration
#define STEERING_INPUT_PIN GPIO_NUM_19

// throttle output MCPWM configuration
#define THROTTLE_OUTPUT_PIN GPIO_NUM_22
#define THROTTLE_OUTPUT_SIGNAL MCPWM0A
#define THROTTLE_OUTPUT_TIMER MCPWM_TIMER_0
#define THROTTLE_OUTPUT_GENERATOR MCPWM_GEN_A

// steering output MCPWM configuration
#define STEERING_OUTPUT_PIN GPIO_NUM_23
#define STEERING_OUTPUT_SIGNAL MCPWM1A
#define STEERING_OUTPUT_TIMER MCPWM_TIMER_1
#define STEERING_OUTPUT_GENERATOR MCPWM_GEN_A

#define PROPSHAFT_GPIO GPIO_NUM_37
#define PROPSHAFT_PCNT PCNT_UNIT_0

#define PERIPHERAL_RESET_GPIO GPIO_NUM_2

#define OPTICAL_SPI_HOST SPI3_HOST
#define OPTICAL_SCLK_GPIO GPIO_NUM_25
#define OPTICAL_MOSI_GPIO GPIO_NUM_26
#define OPTICAL_MISO_GPIO GPIO_NUM_35
#define OPTICAL_CS_GPIO GPIO_NUM_33

#define EINK_SPI_HOST SPI2_HOST
#define EINK_SCLK_GPIO GPIO_NUM_14
#define EINK_MOSI_GPIO GPIO_NUM_13
#define EINK_MISO_GPIO -1
#define EINK_CS_GPIO -1
#define EINK_DC_GPIO GPIO_NUM_0
#define EINK_BUSY_GPIO GPIO_NUM_27

// miscellaneous delays
#define SPLASH_SCREEN_DELAY_MS 8000 // display the splash screen for this period
#define DISPLAY_UPDATE_MS 100 // interval between display updates

SemaphoreHandle_t printMux = NULL;

ScrapDisplay* scrapDisplay = NULL;

esp_adc_cal_characteristics_t adc1Characteristics;

static int16_t sonarDistances[] = { -1, -1, -1 };

static const char* LOG = "scrapmain";

void initGpio() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SYS_CURRENT_ADC1_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(SYS_VOLTAGE_ADC1_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(UI_BUTTONS_ADC1_CHANNEL, ADC_ATTEN_DB_11);
    // 1100 mV is the ESP32 design reference voltage
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100,
            &adc1Characteristics);
}

void updateDisplay(void* param) {
    while (true) {
        vTaskDelay(DISPLAY_UPDATE_MS / portTICK_PERIOD_MS);

/*
        char messageBuffer[80];
        scrapDisplay->setDrawColor(0);
        scrapDisplay->drawBox(36, 12, 4 * 4, 2 * 6);
        scrapDisplay->setFont(u8g2_font_tom_thumb_4x6_tr);
        scrapDisplay->setDrawColor(1);

        sprintf(messageBuffer, "steering %d", steeringPosition);
        scrapDisplay->drawStr(0, 12, messageBuffer);

        sprintf(messageBuffer, "throttle %d", throttlePosition);
        scrapDisplay->drawStr(0, 18, messageBuffer);

        uint32_t currentValue;
        esp_adc_cal_get_voltage(SYS_CURRENT_ADC_CHANNEL, &adc1Characteristics, &currentValue);
        sprintf(messageBuffer, "current  %d mA", currentValue);
        scrapDisplay->drawStr(0, 24, messageBuffer);

        scrapDisplay->sendBuffer();
*/
    }
}

class SonarReceiver : public HCSR04Receiver {
public:
    SonarReceiver(const gpio_num_t echoPin, MCPWMDevice& mcpwmDevice,
            const CaptureSignal& captureSignal) : HCSR04Receiver(echoPin, mcpwmDevice,
            captureSignal) { }

    void echo(uint16_t duration, const HCSR04Trigger* trigger);
};

void SonarReceiver::echo(uint16_t mmDistance, const HCSR04Trigger* trigger) {
    if (mmDistance > 8000) {
        // empirically, over 8m appears to be an "out of range" value
        sonarDistances[trigger->getId()] = -1;
        return;
    }

    sonarDistances[trigger->getId()] = mmDistance;
}

extern "C" void app_main() {
    // initialize the PWM generators as early as possible, to start producing valid R/C signals

    PWMGenerator steeringGenerator(STEERING_OUTPUT_PIN, rcMcpwmDevice, STEERING_OUTPUT_SIGNAL,
            STEERING_OUTPUT_TIMER, STEERING_OUTPUT_GENERATOR);
    RCRepeater steeringRepeater("steering", STEERING_INPUT_PIN, rcMcpwmDevice,
            steeringCaptureSignal, steeringGenerator);

    PWMGenerator throttleGenerator(THROTTLE_OUTPUT_PIN, rcMcpwmDevice, THROTTLE_OUTPUT_SIGNAL,
            THROTTLE_OUTPUT_TIMER, THROTTLE_OUTPUT_GENERATOR);
    RCRepeater throttleRepeater("throttle", THROTTLE_INPUT_PIN, rcMcpwmDevice,
            throttleCaptureSignal, throttleGenerator);

    WiFi wifi(WIFI_SSID, WIFI_PASSWORD);
    Telemetry telemetry;

    printMux = xSemaphoreCreateMutex();

    initGpio();

    // reset peripherals
    gpio_pad_select_gpio(PERIPHERAL_RESET_PIN);
    gpio_set_direction(PERIPHERAL_RESET_PIN, GPIO_MODE_OUTPUT);
    ets_delay_us(250); // ADNS-3080 wants at least 250us from VDD to RESET
    gpio_set_level(PERIPHERAL_RESET_PIN, 1);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(PERIPHERAL_RESET_PIN, 0);
    vTaskDelay(40 / portTICK_PERIOD_MS); // ADNS-3080 wants 35ms after RESET

    PropshaftSensor propshaftSensor(PROPSHAFT_GPIO, PROPSHAFT_PCNT);

    scrapDisplay = new ScrapDisplay(DISPLAY_I2C_PORT, DISPLAY_I2C_SCL_PIN, DISPLAY_I2C_SDA_PIN,
            DISPLAY_I2C_ADDRESS);
    scrapDisplay->test();

    ScrapPlacard scrapPlacard(EINK_SPI_HOST, EINK_SCLK_GPIO, EINK_MOSI_GPIO, EINK_DC_GPIO,
            EINK_BUSY_GPIO);
    scrapPlacard.test();

    SonarReceiver sonarReceiver(SONAR_ECHO_PIN, sonarMcpwmDevice, sonarCaptureSignal);
    HCSR04Trigger sonarRight(0, &sonarReceiver, SONAR_RIGHT_PING_PIN);
    HCSR04Trigger sonarFront(1, &sonarReceiver, SONAR_FRONT_PING_PIN);
    HCSR04Trigger sonarLeft(2, &sonarReceiver, SONAR_LEFT_PING_PIN);

    NXPIMU imu(IMU_I2C_PORT, IMU_I2C_SCL_PIN, IMU_I2C_SDA_PIN);

    ADNS3080 opticalFlow(OPTICAL_SPI_HOST, OPTICAL_SCLK_GPIO, OPTICAL_MOSI_GPIO, OPTICAL_MISO_GPIO,
            OPTICAL_CS_GPIO);

    // a fairly unsophisticated telemetry gathering loop (data is collected synchronously at the
    // same rate for all channels)
    uint64_t telemetrySequence = 0;
    while (true) {
        MotionData motionData = opticalFlow.getMotion();

        InertialData inertialData = imu.getMeasurements();

        uint16_t propshaftPulses = propshaftSensor.getPulseCount();

        uint32_t currentValue;
        esp_adc_cal_get_voltage(SYS_CURRENT_ADC_CHANNEL, &adc1Characteristics, &currentValue);

        uint32_t voltageValue;
        esp_adc_cal_get_voltage(SYS_VOLTAGE_ADC_CHANNEL, &adc1Characteristics, &voltageValue);

        // we don't actually use this right now
        uint32_t buttonValue;
        esp_adc_cal_get_voltage(UI_BUTTONS_ADC_CHANNEL, &adc1Characteristics, &buttonValue);

        sonarRight.ping();
        vTaskDelay(60 / portTICK_PERIOD_MS);
        int16_t rightDistance = sonarDistances[0];

        sonarFront.ping();
        vTaskDelay(60 / portTICK_PERIOD_MS);
        int16_t frontDistance = sonarDistances[1];

        sonarLeft.ping();
        vTaskDelay(60 / portTICK_PERIOD_MS);
        int16_t leftDistance = sonarDistances[2];

        char telemetryData[384];
        sprintf(telemetryData, "seq=%lld,time=%lld,", ++telemetrySequence, esp_timer_get_time());
        if (motionData.isValid) {
            char motionTelemetry[64];
            sprintf(motionTelemetry, "motionX=%d,motionY=%d,sQual=%u,", motionData.deltaX,
                    motionData.deltaY, motionData.sQual);
            strcat(telemetryData, motionTelemetry);
        }
        char telemetryData2[256];
        sprintf(telemetryData2, "accelX=%d,accelY=%d,accelZ=%d,magX=%d,magY=%d,magZ=%d,rotX=%d,"
                "rotY=%d,rotZ=%d,temp1=%u,temp2=%u,distRight=%d,distFront=%d,distLeft=%d,"
                "propshaft=%u,current=%u,voltage=%u", inertialData.accelX, inertialData.accelY,
                inertialData.accelZ, inertialData.magX, inertialData.magY, inertialData.magZ,
                inertialData.rotX, inertialData.rotY, inertialData.rotZ, inertialData.temp1,
                inertialData.temp2, rightDistance, frontDistance, leftDistance, propshaftPulses,
                currentValue, voltageValue);
        strcat(telemetryData, telemetryData2);

        ESP_LOGI(LOG, "%s", telemetryData);
        telemetry.postTelemetry(telemetryData);
    }
}

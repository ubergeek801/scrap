# SCRAP Hardware

## ESP32 pinouts

```
    OLED SCL ----------------+  +---------------------- sonar 3 ping
    OLED SDA -------------+  |  |  +------------------- sonar 2 ping
steering out ----------+  |  |  |  |        +---------- flow MISO
 steering in -------+  |  |  |  |  |        |  +------- sonar echo
throttle out ----+  |  |  |  |  |  |        |  |  +---- UI A/B/C
 throttle in -+  |  |  |  |  |  |  |        |  |  |  +- propshaft RPM
              |  |  |  |  |  |  |  |        |  |  |  |
             21 22 19 23 18 05 10 09 RX TX 35 34 38 37 EN GR 3V
           +----------------------------------------------------+
           |                B              IA IA IA IA          |
           | ESP32-PICO-KIT V4                                  |
           | IA IA DW DW  A  A  W  W  W  W  W BW  W BW          |
           +----------------------------------------------------+
             VP VN 25 26 32 33 27 14 12 13 15 02 04 00 3V GR 5V
              |  |  |  |  |  |  |  |  |  |  |  |  |  |
current draw -+  |  |  |  |  |  |  |  |  |  |  |  |  +- E-Ink D/C
     voltage ----+  |  |  |  |  |  |  |  |  |  |  +---- IMU SDA
   flow SCLK -------+  |  |  |  |  |  |  |  |  +------- peripheral reset
   flow MOSI ----------+  |  |  |  |  |  |  +---------- IMU SCL
    E-Ink CS -------------+  |  |  |  |  +------------- E-Ink MOSI
     flow CS ----------------+  |  |  +---------------- sonar 1 ping
  E-Ink Busy -------------------+  +------------------- E-Ink SCLK

| I = Input-only | A = ADC available | D = DAC available   |
| B = Boot strapping pin | W = ADC available when WiFi off |
```

## System
- Battery voltage: analog input
- Current draw: analog input
- Display: I2C output
- UI input: digital input (pushbutton) x3

## Control
- Steering: PWM input
- Throttle: PWM input
- Steering Servo: PWM output
- ESC: PWM output

## Feedback
- Motor (propshaft) RPM: digital input
- Ground speed: SPI + digital output
- Acceleration/Orientation: I2C input

## Environment
- Proximity: digital output/input x3

## I/O Summary

### Inputs
- Analog inputs 2
- Digital inputs 6
- PWM inputs 2
- I2C inputs 1
- SPI inputs 1

### Outputs
- Analog outputs 0
- Digital outputs 1
- PWM outputs 2
- I2C outputs 1
- SPI outputs 0

## Notes
BRKT-STBC-AGM01
- orange: RST
- yellow: SCL
- green: SDA

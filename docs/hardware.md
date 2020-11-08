# SCRAP Hardware

```
                           IMU RST -+
                         UI ??? -+  |
                    sonar EN -+  |  |
                  sonar 1 -+  |  |  |
               sonar 2 -+  |  |  |  |        +---------- UI ???
            sonar 3 -+  |  |  |  |  |        |  +------- V or I
    OLED/IMU SCL -+  |  |  |  |  |  |        |  |  +---- UI ???
 OLED/IMU SDA -+  |  |  |  |  |  |  |        |  |  |  +- Hall (propshaft RPM)
               |  |  |  |  |  |  |  |        |  |  |  |
              21 22 19 23 18 05 10 09 RX TX 35 34 38 37 EN GR 3V
            +----------------------------------------------------+
            |                                I  I  I  I          |
            |                                                    |
            +----------------------------------------------------+
              VP VN 25 26 32 35 27 14 12 13 15 02 04 00 3V GR 5V
                     |  |  |  |  |  |     |        |
        steering in -+  |  |  |  |  |     |        +- steering or throttle out
           throttle in -+  |  |  |  |     +---------- steering or throttle out
                flow SCLK -+  |  |  |
                   flow MOSI -+  |  |
                      flow MISO -+  |
                          flow CS? -+
```

## System
- Battery voltage: analog input
- Current draw: analog input
- Display: I2C output
- UI input: digital input x3

## Control
- Steering: PWM input
- Throttle: PWM input
- Steering Servo: PWM output
- ESC: PWM output

## Feedback
- Motor RPM: digital input
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

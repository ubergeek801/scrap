# SCRAP
_scale car racing automation platform_

## Overview

SCRAP is initially an experiment in implementing, in a radio-controlled car, "driver assists" now
ubiquitous in full-scale vehicles. The plan is to start simple, with straight-line dynamics:
- traction control: avoiding excessive wheelspin and unintended direction change under standing-start
  acceleration
- antilock braking ("ish"): maintaining a straight line under hard braking (although R/C motors
  generally aren't capable of locking the wheels at speed)

More advanced capability would include:
- obstacle avoidance: applying braking and/or steering overrides when an impending collision is
  predicted
- general stability control: attempting to correct the difference between the driver's intended
  direction and the vehicle's actual direction--particularly, avoiding a spin

These goals entail the development of a vehicle dynamics model, through the use of various sensors:
- throttle and steering inputs
- vehicle speed measurement (directly, such as using an optical flow sensor; and indirectly, such as
  by measuring the rotational velocity of a wheel or something mechanically connected to it)
- inertial measurement using accelerometers, gyros, etc.

Assuming success in these areas, future endevaors may explore applications of machine learning, such
as:
- course following: using a guide, such as a tape line, to follow some predetermined course
- course memorization: "replaying" human-driven laps around a course, possibly with some degree of
  correction and/or adaptation
- course navigation: navigating a course autonomously, without previous course-specific training or
  human assistance

These latter goals are largely aspriational and tentative, as there are already a number of machine
learning-oriented projects, competitions, etc. focusing on scale vehicles.

## The Platform

In general, the SCRAP platform favors inexpensive and widely available components, and will try to
achieve as much as possible with these unless it becomes clear that a more exotic solution is
needed. LIDAR, for example, is the current state of the art for obstacle detection and ranging,
but is substantially more expensive than commonplace SONAR sensors, which just might suit SCRAP's
purposes.

### Vehicle Platform

Originally, it was envisioned that the project founder's 1/10 touring car chassis (HPI RS4 Pro 2)
would serve as the development prototype; however, it is hopelessly lost in storage. A suitable
alternative was found in the Tamiya TT-02, which is relatively inexpensive, readily available and
reasonably durable. Furthermore, being highly upgradeable, the TT-02 will likely exceed SCRAP's own
performance capabilities for the foreseeable future.

### Sensor Platform

The relatively large number and variety of sensors to be employed demands a platform featuring
robust I/O capabilities. Such capabilities, as well as fairly impressive (for a microcontroller)
CPU power, are found in the Espressif ESP32 family of devices. SCRAP currently employs the
ESP32-PICO-KIT, which despite its compact size, offers a plethora of I/O pins as well as WiFi
capability.

TODO: enumerate sensors

### Computing Platform

As the first phase of the SCRAP project is focused on sensor data acquisition and fusion, it is
expected that the attendant computing tasks (for example, numerical integration) should require
only modest CPU capability. The ESP32 used in the sensor platform should be up to the task. When
(or if) SCRAP advances to incorporate machine learning, it is likely that a NPU (such as Google's
Coral or NVIDIA's Jetson) will be introduced alongside the ESP32, with the latter continuing to
function as an I/O coprocessor.

Since the inception of SCRAP, the PJRC Teensy 4.x boards have arrived on the market, boasting
substantially greater CPU power than the ESP32, as well as comparable I/O capability, with the only
apparent disadvantage of lacking onboard WiFi. Teensy 4.x would certainly be considered if more
computational grunt is needed to process sensor input.

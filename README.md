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

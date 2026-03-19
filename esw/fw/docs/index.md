# MRover Embedded Software

The embedded software team (ESW) of MRover writes the driver code that allows the other programming
subteams to utilize the electronic equipment on the rover. ESW works primarily with libraries in C,
C++, and Python to abstract the functions needed by the other teams for easy use. To do this we use
different communication protocols such as CAN and I2C to transmit and receive data from sensors,
motors, and various other components.

New to ESW? Check out the [getting started page](getting-started/intro.md) for developer setup and starter projects.

## Overview

ESW is split into two groups: Controls and Telemetry. The main difference is simply the subject
matter of their projects.

Controls handles the control systems on the rover, dealing mainly with brushless and brushed DC
motors. We currently use moteus-r4.11 and moteus-n1 brushless controllers from mjbots, however we
aim to replace these entirely with our own custom brushless controller. Our brushed controllers are
already fully custom and developed by us.

Telemetry handles the data and sensing systems on the rover, dealing mainly with the science system.
<!---TODO: ADD MORE INFO-->

# PID-Stabilisation-Glider 

# Self-Stabilising Glider

A model glider that uses an MPU6050 and PID control to automatically correct its pitch and roll.

The system uses four servos to control the glider's ailerons and elevators.

A Processing program also displays the glider's real-time position and orientation.

## Features

- MPU6050 attitude sensing
- PID pitch and roll control
- Four-servo control
- Arduino-based control system
- Real-time Processing visualisation

## Hardware

- Arduino
- MPU6050
- 4 × servos
- Model glider

## Software

- Arduino / C++
- Processing


The system was tested by manually disturbing the glider from its neutral
orientation and observing the response of the control system.

The controller responds to changes in pitch and roll by commanding the
appropriate control surfaces to return the glider towards its target
orientation.

Example serial output:

```text
Pitch: 4.21 | Roll: -2.83 | Pitch PID: -8.42 | Roll PID: 5.66
Pitch: 2.74 | Roll: -1.61 | Pitch PID: -4.32 | Roll PID: 3.21
Pitch: 0.91 | Roll: -0.52 | Pitch PID: -1.82 | Roll PID: 1.04

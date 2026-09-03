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

## Visualisation

![Processing Visualisation](images/processing-visualisation.png)

## Glider

![Glider](images/glider.jpg)

## Project Structure

```text
Self-Stabilising-Glider/
├── Arduino/
│   └── Arduino-MPU6050.ino
├── Processing/
│   └── GliderVisualisation.pde
├── images/
├── README.md
└── .gitignore

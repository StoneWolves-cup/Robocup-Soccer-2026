# Stone Wolves — RoboCup Soccer 2026
<hr>
<p align="center">
  <img src="images/robot.jpg" width="700" alt="Stone Wolves RoboCup Soccer Robot">
</p>

<p align="center">
  <strong>RoboCup Soccer Infrared — 2026</strong><br>
  Autonomous Robotics | Embedded Systems | C++
</p>
<hr>
## Overview

<p>This repository contains the software developed by Stone Wolves for the RoboCup Soccer Infrared 2026 competition.
The project focuses on the development of autonomous robotic systems capable of detecting the ball, navigating the field, maintaining orientation, and executing movement strategies during matches.
The firmware is developed primarily in C++ and runs on ESP32-based embedded systems.</p>  

<hr>

## Robot Systems

<p>The software is organized into different modules responsible for the robot's operation.
Movement System – Controls the robot's motors and omnidirectional movement.
Motor Control – Implements motor operation and movement commands.
Orientation System – Uses the BNO055 sensor to obtain orientation data and support directional control.
PID Control – Provides feedback-based control for maintaining the desired movement and orientation.
These modules are organized to facilitate development, testing, and future improvements.</p>  

<hr>

## Development Environment
<p>
Programming Language: C++
Microcontroller: ESP32
Framework: Arduino
IDE: Visual Studio Code
Build System: PlatformIO
Version Control: Git
</p>

<hr>

## Hardware Components

<p>ESP32 microcontrollers
BNO055 orientation sensor
HC-SR04 ultrasonic sensors
TSSP4038 infrared sensors
Motor control electronics</p>  

<hr>

## Repository Structure

Robocup-Soccer-2026/
│
├── include/
│   ├── BNO055.h
│   ├── Motores.h
│   ├── Omni.h
│   └── README
│
├── lib/
│   └── README
│
├── src/
│   ├── main.cpp
│   │
│   └── Movement/
│       ├── MotorMain/
│       │   ├── MotorMain.ino
│       │   └── Motores.cpp
│       │
│       ├── Omni/
│       │   └── Omni.cpp
│       │
│       └── mainBNO/
│           └── BNO055.cpp
│
├── test/
│   └── README
│
├── platformio.ini
└── .gitignore

<hr>

## Getting Started

### Requirements

<p>Before building the firmware, make sure you have installed:
Visual Studio Code
PlatformIO IDE extension
Git
ESP32 platform support
</p>  
### Clone the repository

<p>git clone -b national https://github.com/StoneWolves-cup/Robocup-Soccer-2026.git
cd Robocup-Soccer-2026</p>

### Build and Upload

<p>Open the repository folder in Visual Studio Code.
Open the PlatformIO extension.
Select the appropriate project environment.
Connect the ESP32 to your computer.
Build the firmware.
Upload the program to the board.
The board configuration and build environment are defined in platformio.ini.</p>

## Documentation

<p>Additional technical information about the project can be found in the repository and its documentation files.
The documentation covers the robot's development, electronic systems, mechanical design, and software architecture.</p>

<hr>

## Notes

<p>This software is developed specifically for the Stone Wolves competition robot.
Hardware configurations, GPIO assignments, sensor addresses, and communication settings depend on the corresponding electronic implementation.
Modifications may be required when adapting the firmware to different hardware configurations.</p>

<hr>

## Team

<p>Stone Wolves
RoboCup Soccer Infrared — 2026
Ribeirão Preto, São Paulo, Brazil

Repository: Stone Wolves — Robocup Soccer 2026</p>

<p align="center">
  Developed by <strong>Stone Wolves</strong> for RoboCup Soccer 2026.
</p>

# Embedded Robotics: Line-Following-Robot

## Overview

This project, undertaken as part of the "Electronics for Embedded Systems" course, involves the design and implementation of a motorized robot equipped with a set of sensors and display systems. The robot is designed to operate in two distinct modes:

1. **Line-Following (Automatic) Mode**: The robot autonomously follows a designated path, while navigating around obstacles and intersections.
2. **Soccer (Manual) Mode**: The robot participates in a soccer game, requiring agile maneuvers and precise control.

This project showcases practical applications of embedded systems concepts, including electronics, component integration, and real-time programming. 

## Contributors

- **Achraf EL FAIQ**
- **Lisa LEVASSEUR**
- **Tom POGET**
- **Yassine ZAHOU**

## Components

The robot integrates a range of components to achieve its functionality:

- **Raspberry Pi**
- **Line Followers (TCRT5000)**
- **Active Buzzer**
- **Ultrasonic Distance Sensor (HC-SR04)**
- **Infrared Sensor**
- **I2C Interface**
- **LCD Screen (16x2)**
- **Push Button**
- **L298N Motor Driver**
- **Two Motors**
- **Two Batteries**

## Electrical Schematics

Our electrical design strategically integrates the components to ensure effective functionality. Given a malfunctioning Raspberry Pi towards the end of the project, we optimized connections by consolidating all grounds and utilizing the I2C interface for the LCD screen to avoid power issues and minimize GPIO usage.

## Component Choices and Techniques

### Motors

We employed the L298N H-bridge motor driver for bidirectional motor control, utilizing Pulse Width Modulation (PWM) to adjust motor speed precisely—critical for accurate line-following.

### Active Buzzer

An active buzzer provides auditory feedback for startup notifications and obstacle alerts.

### IR Sensor and Remote Control

An IR receiver interprets signals from a standard remote control, facilitating remote operation and mode switching.

### Ultrasonic Sensor

The HC-SR04 ultrasonic sensor measures distances using echolocation to detect obstacles and aid navigation.

### LCD Screen

The 16x2 LCD screen displays key information such as obstacle distance and intersection status, connected via the I2C protocol to simplify wiring and reduce power concerns.

### Line Followers

TCRT5000 line followers are used to detect surface reflectance, guiding the robot along a track with adjustable sensitivity through a potentiometer.

## Multithreading

To manage real-time sensor data, remote control commands, and obstacle detection, we implemented multithreading. This ensures responsive operation and efficient handling of concurrent tasks.

## Code Overview

### Main Program

The main program handles initialization of signal handlers, GPIO configurations, and various components. It includes threads for:

- **Obstacle Detection**
- **Line Following**
- **Remote Control Management**

The robot operates in two modes:

- **Automatic Mode**: Autonomously follows lines and navigates intersections.
- **Manual Mode**: Controlled via remote, allowing movement and speed adjustments.

### Motor and Line Following

The `motor_decision` function interprets sensor data to guide the robot's movements based on line detection and obstacle avoidance.

### Remote Control

Remote control functions include initialization, data retrieval, and command execution based on remote signals.

### Buzzer

Two functions manage the buzzer to produce startup sounds and operational alerts.

### Ultrasonic Sensor

The `disMeasure` function calculates distances using the ultrasonic sensor for obstacle detection.

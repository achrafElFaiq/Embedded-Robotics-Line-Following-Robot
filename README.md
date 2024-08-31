# Embedded Robotics: Line-Following-Robot

## Overview

This project, undertaken as part of the "Electronics for Embedded Systems" course, involves the design and implementation of a motorized robot equipped with a set of sensors and display systems. The robot is designed to operate in two distinct modes:

1. **Line-Following (Automatic) Mode**: The robot follows a line while detecting obstacles and intersections on the path. This mode integrates a complex combination of sensors and control logic to ensure precise and responsive navigation.

2. **Soccer (Manual) Mode**: The robot engages in a soccer match against an opposing team. This mode requires precise control of the robot to maneuver effectively during the game.

The project demonstrates practical application of various technical and conceptual skills acquired during the semester in embedded systems electronics. It involved addressing numerous challenges in electronics and adhering to a comprehensive set of objectives:

- Understanding the basics of electronics
- Interpreting and using component documentation
- Assembling a functional and safe electronic setup
- Developing a functional program
- Utilizing a Raspberry Pi effectively

The following sections provide detailed insights into the project tasks, electrical schematics, component choices, and the code implemented for robot operation.

## Project Contributors

- **Achraf EL FAIQ**
- **Lisa LEVASSEUR**
- **Tom POGET**
- **Yassine ZAHOU**

## Components Used

- **Raspberry Pi**
- **Line Followers (TCRT5000)**
- **Buzzer**
- **Ultrasonic Distance Sensor (HC-SR04)**
- **Infrared Sensor**
- **I2C Interface**
- **LCD Screen (16x2)**
- **Push Button**
- **L298N Motor Driver**
- **Two Motors**
- **Two Batteries**

## Electrical Schematics

The electrical design integrates the components with careful attention to minimizing GPIO usage due to a malfunctioning Raspberry Pi used late in the project. This included consolidating all grounds and using the I2C interface for the LCD to optimize connections and avoid power issues.

## Component Choices and Techniques

### Motors

The L298N H-bridge motor driver was selected for bidirectional control of the motors. Pulse Width Modulation (PWM) is used to regulate motor speed, allowing precise control crucial for line-following tasks.

### Active Buzzer

An active buzzer was used to produce sound signals for startup notifications and obstacle alerts.

### IR Sensor and Remote Control

An IR receiver decodes signals from a standard infrared remote control, enabling remote operation and mode switching.

### Ultrasonic Sensor

The HC-SR04 ultrasonic sensor measures distances using echolocation to detect obstacles and assist with navigation.

### LCD Screen

The 16x2 LCD screen displays information such as obstacle distance and intersection status. The I2C protocol simplifies the connection to the microcontroller.

### Line Followers

TCRT5000 line followers detect surface reflectance to guide the robot along a track, using a potentiometer for fine adjustment.

## Multithreading

Multithreading was implemented to handle real-time sensor data processing, remote control commands, and obstacle detection concurrently. This approach was necessary to avoid delays and ensure responsive operation.

## Code Overview

### Main Program

The main program initializes signal handlers, GPIO configurations, and various components. It creates threads for obstacle detection, line following, and remote control management. The robot operates in two modes:

- **Automatic Mode**: Uses sensors to follow a line and navigate intersections.
- **Manual Mode**: Allows control via remote, with actions including movement and speed adjustments.

### Motor and Line Following Decisions

The `motor_decision` function interprets sensor data to control the robot’s movements based on detected lines and obstacles.

### Remote Control

The remote control functions include initialization, data retrieval, decoding, and conversion from binary to decimal values to execute remote commands.

### Buzzer

Two functions handle the buzzer to produce startup sounds and alerts during operation.

### Ultrasonic Sensor

The `disMeasure` function calculates distances using the ultrasonic sensor, essential for obstacle avoidance.

### LCD Screen

The LCD displays real-time information about the robot's environment, including detected distance and intersection status.

# Arduino RC Car

A remote-controlled car project using Arduino UNO and L298N Motor Controller.

## Overview

This project implements a remote-controlled car with two motors:
- One motor for driving (propulsion)
- One motor for steering

## Hardware Requirements

- Arduino UNO
- L298N Motor Controller
- Two DC motors
- RC Receiver
- Power supply for motors
- Chassis and mechanical components
- Connecting wires

## Pin Configuration

### L298N Motor Controller Connections
- **Drive Motor:**
    - ENA: Pin 5 (PWM speed control)
    - IN1: Pin 7 (Direction control)
    - IN2: Pin 8 (Direction control)

- **Steering Motor:**
    - ENB: Pin 6 (PWM speed control)
    - IN3: Pin 9 (Direction control)
    - IN4: Pin 10 (Direction control)

### RC Receiver Connections
- Forward signal: A0
- Backward signal: A1
- Left signal: A2
- Right signal: A3

## Features

- Independent control of drive and steering motors
- Variable speed control for both motors
- Serial monitor debugging and control
- RC receiver input processing
- Safety features (motors stop at startup)

## How It Works

The Arduino reads signals from an RC receiver and translates them into motor movements:
- Forward/backward signals control the drive motor
- Left/right signals control the steering motor
- Signal strength determines motor speed

## Serial Commands

For testing without the RC receiver, send these characters via Serial Monitor:
- 'F': Drive forward
- 'B': Drive backward
- 'L': Steer left
- 'R': Steer right
- 'S': Stop drive motor
- 'C': Center steering

## Future Improvements

- Add position feedback for steering (potentiometer or limit switches)
- Implement battery monitoring
- Add headlights and taillights
- Incorporate obstacle detection

## Getting Started

1. Connect the hardware according to the pin configuration
2. Upload the provided code to your Arduino
3. Power on the system
4. Control using your RC transmitter or serial commands

## Notes

For accurate steering centering, consider adding position feedback such as a potentiometer or limit switches.
# ESP32 RC Car Controller

A Wi-Fi controlled RC car using ESP32 microcontroller operating in access point mode.

## Overview

This project implements a web-based remote control system for a model car using an ESP32 microcontroller and L298N motor driver. The system creates its own Wi-Fi access point for direct control.

## Features

- Standalone Wi-Fi access point mode
- Web-based control interface
- WebSocket communication for low-latency control
- Basic movement controls (forward, backward, left, right)
- Simple configuration

## Hardware Requirements

- ESP32 development board
- L298N motor driver
- DC motors (2x)
- Power supply for motors
- RC car chassis
- Jumper wires
- Battery pack

## Wiring Diagram

| ESP32 Pin | L298N Pin | Function |
|-----------|-----------|----------|
| GPIO14    | ENA       | Enable Motor A |
| GPIO27    | IN1       | Motor A Direction 1 |
| GPIO26    | IN2       | Motor A Direction 2 |
| GPIO25    | IN3       | Motor B Direction 1 |
| GPIO33    | IN4       | Motor B Direction 2 |
| GPIO32    | ENB       | Enable Motor B |
| 5V        | 5V        | Logic power |
| GND       | GND       | Common ground |

## Software Dependencies

- ESP32 Arduino Core
- WebServer library
- WebSocketsServer library
- WiFi library

## Setup and Installation

1. Install the Arduino IDE
2. Install the ESP32 board support package in Arduino IDE
3. Install required libraries:
   - WebSocketsServer (by Markus Sattler)
   - ESP32 DNSServer, WebServer (included in ESP32 Arduino Core)
4. Clone this repository or download the code
5. Open the `ESP32-RC.ino` file in Arduino IDE
6. Modify the access point credentials if needed
7. Connect your ESP32 board via USB
8. Select the correct board and port in Arduino IDE
9. Upload the sketch to your ESP32

## Configuration

### Wi-Fi Settings

The default configuration creates an access point with:
- SSID: `ESP32_RC_Car`
- Password: `12345678`
- IP Address: `192.168.4.1`

You can modify these settings in the code if needed:
```cpp
const char* ap_ssid = "ESP32_RC_Car"; 
const char* ap_password = "12345678";
```

## Usage

1. Power on the ESP32 RC car
2. Connect to the "ESP32_RC_Car" Wi-Fi network using the password "12345678"
3. Open a web browser and navigate to `http://192.168.4.1`
4. Use the on-screen controls to operate the car:
   - Forward: Move car forward
   - Backward: Move car backward
   - Left: Turn car left
   - Right: Turn car right
   - Stop: Stop all motors

## Command Codes

The system uses simple command codes to control movement:
- `0`: Stop
- `1`: Forward
- `2`: Backward
- `4`: Left
- `8`: Right

## Troubleshooting

- **Motors not responding:**
  - Check wiring connections
  - Ensure sufficient battery power
  - Verify motor driver is receiving power
- **Cannot connect to Wi-Fi:**
  - Reset the ESP32
  - Verify correct password is being used
  - Ensure you're within range of the access point
- **Web page not loading:**
  - Confirm you're connected to the correct Wi-Fi network
  - Try accessing by IP address (192.168.4.1)
  - Clear browser cache or try a different browser

## License

This project is released under the GNU General Public License v3.0.

## Contributing

Contributions to improve the code or documentation are welcome. Please feel free to submit a pull request.

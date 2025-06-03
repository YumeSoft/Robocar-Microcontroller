# 🚗 ESP32 Robocar Microcontroller

A comprehensive collection of ESP32-based robotic car projects featuring remote control and autonomous driving capabilities with advanced motor control and sensor integration.

## 📋 Project Overview

This repository contains two main projects:

### 🎮 **ESP32 RC Car with Robotic Arm**

- **Web-based Control Interface**: Responsive HTML/CSS interface with real-time WebSocket communication
- **6-DOF Robotic Arm**: Complete servo-controlled arm with base, shoulder, elbow, and gripper
- **Advanced Motor Control**: Differential drive system with precise PWM control
- **Mobile-Responsive Design**: Touch-friendly controls optimized for smartphones and tablets
- **Automated Shooting System**: Pre-programmed ball shooting sequence with servo coordination

### 🤖 **ESP32 Self-Driving Car**

- **Autonomous Line Following**: Multi-sensor array for precise line tracking
- **Intelligent Navigation**: Adaptive turning algorithms with hard/soft turn detection
- **Safety Systems**: Stair detection with configurable timeout mechanisms
- **Debug & Monitoring**: Real-time sensor data and system status via Serial
- **Recovery Algorithms**: Automatic line recovery when sensors lose track

---

## 🏗️ Project Structure

```
Robocar-Microcontroller/
├── ESP32-RC-Car/
│   ├── ESP32-RC.ino          # Main RC car with WebSocket control
│   ├── main.ino              # Alternative AsyncWebServer implementation
│   └── index.h               # Embedded HTML interface
├── Self-Driving-Car/
│   └── ESP32-SelfDriving.ino # Autonomous line-following system
├── LibraryConflictSolver.h   # Resolves WebServer library conflicts
└── README.md                 # Project documentation
```

---

## 🛠️ Hardware Requirements

### 🔧 Common Components

- **ESP32 DevKit V1** (or compatible development board)
- **L298N Motor Driver Module** with heat sinks
- **2x DC Gear Motors** (6V-12V, with wheels and mounting brackets)
- **7.4V Li-Po Battery** (2200mAh+ recommended) with JST connector
- **Chassis Platform** (acrylic, aluminum, or 3D printed)
- **Jumper Wires** (male-to-male, male-to-female)
- **Breadboard or PCB** for connections
- **Power Switch** and LED indicator

### 🎮 RC Car Additional Components

- **6x SG90 Servo Motors** (or equivalent 9g servos)
- **Robotic Arm Kit** (3D printed or aluminum brackets)
- **Servo Horn Set** (various shapes for arm joints)
- **Ball Launcher Mechanism** (custom or commercial)
- **Status LED** (optional, for connection indication)

### 🤖 Self-Driving Car Additional Components

- **4x TCRT5000 Line Sensors** (or IR photointerrupter modules)
- **1x Additional Sensor** for stair/edge detection
- **Sensor Mounting Bar** (adjustable height bracket)
- **Reflective Line Tape** (white on black or vice versa)
- **Track Material** (poster board, vinyl, or painted surface)

---

## 📌 Detailed Pin Configuration

### 🎮 RC Car Pinout

#### 🔌 Motor Driver (L298N) Connections
| L298N Pin | ESP32 Pin | Function | Wire Color (Suggested) |
|-----------|-----------|----------|----------------------|
| ENA | GPIO 18 | Right motor PWM speed | Orange |
| IN1 | GPIO 19 | Right motor direction A | Yellow |
| IN2 | GPIO 21 | Right motor direction B | Green |
| ENB | GPIO 15 | Left motor PWM speed | Blue |
| IN3 | GPIO 4 | Left motor direction A | Purple |
| IN4 | GPIO 2 | Left motor direction B | Gray |
| VCC | 5V | Logic power | Red |
| GND | GND | Ground reference | Black |

#### 🦾 Servo Motor Assignments
| Servo Function | ESP32 Pin | Servo Color | Range | Initial Position |
|---------------|-----------|-------------|-------|-----------------|
| Base Rotation | GPIO 27 | Red | 0-180° | 90° (Center) |
| Shoulder Joint | GPIO 26 | Orange | 0-180° | 90° (Horizontal) |
| Elbow Joint | GPIO 25 | Yellow | 0-180° | 90° (Straight) |
| Gripper Control | GPIO 33 | Green | 0-180° | 90° (Half open) |
| Launcher Servo 1 | GPIO 32 | Blue | 0-180° | 0° (Ready) |
| Launcher Servo 2 | GPIO 21 | Purple | 0-180° | 0° (Ready) |

#### 💡 Status Indicators
| Component | ESP32 Pin | Function | Connection |
|-----------|-----------|----------|------------|
| Status LED | T2 (GPIO 2) | WiFi/Connection status | LED + 220Ω resistor |

### 🤖 Self-Driving Car Pinout

#### 🔌 Motor Driver (L298N) Connections
| L298N Pin | ESP32 Pin | Function | Notes |
|-----------|-----------|----------|-------|
| ENA | GPIO 5 | Right motor PWM | Higher current capability |
| ENB | GPIO 18 | Left motor PWM | No PWM removal needed |
| IN1 | GPIO 2 | Right motor direction A | Digital output |
| IN2 | GPIO 4 | Right motor direction B | Digital output |
| IN3 | GPIO 16 | Left motor direction A | Digital output |
| IN4 | GPIO 17 | Left motor direction B | Digital output |

#### 👁️ Line Sensor Array
| Sensor Position | ESP32 Pin | Function | Mounting Position |
|----------------|-----------|----------|------------------|
| Leftmost | GPIO 13 | Hard left detection | Far left of chassis |
| Left Inner | GPIO 12 | Soft left correction | Left of center |
| Right Inner | GPIO 14 | Soft right correction | Right of center |
| Rightmost | GPIO 27 | Hard right detection | Far right of chassis |
| Stair Sensor | GPIO 26 | Edge/drop detection | Front center, angled down |

---

## ⚡ Advanced Features

### 🎮 RC Car Capabilities

#### **Real-Time Web Control**
- **Responsive Design**: Auto-adapts to screen size (320px - 1920px+)
- **Touch Optimized**: Large buttons with visual feedback
- **Live Servo Feedback**: Real-time angle display (0-180°)
- **Connection Status**: Visual indicators for WiFi and WebSocket status
- **Low Latency**: <50ms response time over local WiFi

#### **Robotic Arm Intelligence**
- **Smooth Movement**: Servo interpolation prevents jerky motion
- **Coordinated Actions**: Multi-servo sequences for complex tasks
- **Safety Limits**: Software-enforced range restrictions
- **Position Memory**: Remembers last position after power cycle

#### **Motor Control System**
- **Differential Drive**: Independent left/right motor control
- **Speed Ramping**: Gradual acceleration/deceleration
- **Direction Logic**: Proper H-bridge control sequence
- **PWM Optimization**: 8-bit resolution (0-255) speed control

### 🤖 Self-Driving Features

#### **Advanced Line Following**
- **Multi-Sensor Fusion**: 4-sensor array for precise tracking
- **Adaptive Algorithms**: Different responses for tight vs wide turns
- **Speed Modulation**: Automatic speed adjustment based on turn severity
- **Line Loss Recovery**: Continues last known direction when line is lost

#### **Safety & Navigation**
- **Stair Detection**: Prevents falls with configurable timeout (default: 20s)
- **Finish Line Recognition**: Stops when all sensors detect line simultaneously
- **Emergency Recovery**: Systematic search pattern when completely lost
- **Debug Monitoring**: Real-time sensor values via Serial (115200 baud)

#### **Intelligent Turning Logic**
```cpp
// Turn Classification System
- Soft Turn: Inner sensor triggered (slight course correction)
- Hard Turn: Outer sensor triggered (sharp direction change)
- Recovery Turn: All sensors white (systematic search)
- Emergency Stop: All sensors black (finish line detection)
```

---

## 🚀 Detailed Setup Guide

### 1. 🔧 Hardware Assembly

#### **Chassis Preparation**
1. Mount ESP32 on chassis with standoffs
2. Install L298N motor driver with proper ventilation
3. Secure battery compartment with easy access for charging
4. Plan wire routing to avoid motor interference

#### **Motor Installation**
1. Mount motors with proper alignment for straight tracking
2. Ensure wheels are parallel and balanced
3. Test motor rotation direction before final connections
4. Add wheel encoders if precision control is needed

#### **Sensor Mounting (Self-Driving)**
1. Mount sensors 5-10mm above track surface
2. Ensure sensors are perpendicular to travel direction
3. Space sensors evenly across chassis width (80-120mm total)
4. Angle stair sensor downward 30-45° from horizontal

### 2. 📚 Software Installation

#### **Arduino IDE Setup**
```bash
# Add ESP32 Board Manager URL:
https://dl.espressif.com/dl/package_esp32_index.json

# Install ESP32 Board Package (v2.0.0+)
# Select: ESP32 Dev Module
```

#### **Required Libraries**
```cpp
// Install via Library Manager:
ESP32Servo v0.12.0+          // Servo control
AsyncTCP v1.1.1+             // TCP connections  
ESPAsyncWebServer v1.2.3+    // Web server
WebSocketsServer v2.3.6+     // Real-time communication
WiFi (built-in)              // Network connectivity
```

### 3. 📤 Code Upload Process

#### **Pre-Upload Checklist**
- [ ] ESP32 connected via USB
- [ ] Correct board selected (ESP32 Dev Module)
- [ ] Upload speed set to 921600
- [ ] Flash frequency: 80MHz
- [ ] Flash mode: DIO
- [ ] Flash size: 4MB

#### **Upload Steps**
1. Open appropriate `.ino` file in Arduino IDE
2. Verify code compiles without errors
3. Hold BOOT button while clicking upload (if required)
4. Monitor Serial output for successful initialization

### 4. ⚙️ Configuration & Calibration

#### **RC Car WiFi Setup**
```cpp
// Network Configuration (modify in code)
const char* ap_ssid = "Mat ngu trien mien";     // Access Point name
const char* ap_password = "monggiadinhanlanh";  // Password (8+ chars)
IPAddress ap_local_IP(192, 168, 4, 1);         // Static IP
```

#### **Self-Driving Calibration**
1. **Sensor Testing**: Use Serial Monitor to verify sensor readings
2. **Speed Tuning**: Adjust `baseSpeed`, `turnSpeed`, `hardTurnSpeed`
3. **Line Testing**: Test on actual track with proper lighting
4. **Timeout Setting**: Configure stair detection duration

---

## 🎮 Comprehensive Usage Guide

### RC Car Operation

#### **🌐 Web Interface Access**
1. Power on the ESP32 and wait for initialization (LED indicators)
2. Connect device to WiFi network "Mat ngu trien mien"
3. Open web browser and navigate to `http://192.168.4.1`
4. Wait for interface to load and WebSocket connection to establish

#### **🕹️ Control Interface Elements**

**Servo Control Panel:**
- **Sliders**: Drag to set servo angles (0-180°)
- **Real-time Display**: Shows current angle in degrees
- **Smooth Movement**: Servos move gradually to prevent damage

**Vehicle Control:**
- **Arrow Buttons**: Touch and hold for movement
- **Directional Logic**: Forward/backward, left/right turning
- **Speed Control**: Fixed speeds for consistent performance

**Special Functions:**
- **SHOOT BALL**: Automated sequence using Servo5 and Servo6
- **Connection Status**: Visual feedback for system health

#### **🔌 Direct WebSocket Commands**
```javascript
// Vehicle Movement (value: "1" = start, "0" = stop)
websocket.send("Forward,1");    // Move forward
websocket.send("Backward,1");   // Move backward  
websocket.send("Left,1");       // Turn left
websocket.send("Right,1");      // Turn right

// Servo Control (value: 0-180 degrees)
websocket.send("Base,45");      // Rotate base to 45°
websocket.send("Shoulder,120"); // Lift shoulder to 120°
websocket.send("Elbow,60");     // Bend elbow to 60°
websocket.send("Gripper,0");    // Close gripper fully

// Special Functions
websocket.send("RotateServos,1"); // Execute shooting sequence
```

### Self-Driving Car Operation

#### **🛤️ Track Requirements**
- **Line Width**: 20-25mm for optimal detection
- **Surface**: Matte finish to reduce glare
- **Contrast**: High contrast between line and background
- **Curves**: Minimum radius 200mm for stable following
- **Lighting**: Even, diffused lighting without shadows

#### **📊 Serial Monitor Commands**
```bash
# Stair Detection Timeout (seconds)
timeout:30          # Set 30-second timeout
timeout:10          # Set 10-second timeout  
timeout:0           # Disable timeout (not recommended)

# Runtime Information
# System automatically prints:
# - Sensor values every cycle
# - Motor commands and speeds
# - Turn decisions and logic
# - Error states and recovery actions
```

#### **🔍 Sensor Interpretation**
```cpp
// Digital Sensor Values:
1 = Black line detected (sensor over line)
0 = White surface (sensor over background)

// Sensor Patterns and Actions:
"0001" = Hard right turn needed
"0010" = Soft right correction  
"0100" = Soft left correction
"1000" = Hard left turn needed
"0110" = Perfect alignment, go straight
"1111" = Finish line detected, stop
"0000" = Line lost, recovery mode
```

---

## ⚙️ Advanced Configuration

### 🎮 RC Car Customization

#### **Performance Tuning**
```cpp
// Motor Speed Configuration
#define MOTOR_BASE_SPEED 250    // Normal driving (0-255)
#define MOTOR_SLOW_SPEED 160    // Precise movements
#define MOTOR_TURN_SPEED 200    // Turning speed

// Servo Movement Settings  
#define SERVO_STEP_SIZE 3       // Degrees per update cycle
#define SERVO_UPDATE_INTERVAL 5 // Milliseconds between updates
#define SERVO_MIN_PULSE 544     // Minimum pulse width (microseconds)
#define SERVO_MAX_PULSE 2400    // Maximum pulse width (microseconds)

// Network Configuration
#define WEBSOCKET_PORT 81       // WebSocket communication port
#define HTTP_PORT 80            // Web server port
#define AP_MAX_CONNECTIONS 4    // Maximum simultaneous connections
```

#### **Custom Control Schemes**
```cpp
// Add custom movement patterns
void customMovementPattern() {
  // Example: Figure-8 pattern
  CAR_turnLeft();
  delay(1000);
  CAR_moveForward();
  delay(2000);
  CAR_turnRight();
  delay(2000);
  // Continue pattern...
}

// Add servo choreography
void customServoSequence() {
  moveServoSmoothly(0, 45);   // Base to 45°
  delay(500);
  moveServoSmoothly(1, 120);  // Shoulder up
  delay(500);
  moveServoSmoothly(3, 0);    // Close gripper
}
```

### 🤖 Self-Driving Car Fine-Tuning

#### **Algorithm Parameters**
```cpp
// Speed Control Matrix
int baseSpeed = 255;           // Maximum forward speed
int turnSpeed = 190;           // Inner wheel speed during turns  
int hardTurnSpeed = 80;        // Aggressive turn speed
int reducedTurnSpeed = 150;    // Outer wheel during hard turns

// Timing Constants  
unsigned long MIN_TURN_TIME = 100;        // Minimum turn duration (ms)
unsigned long STAIR_TIMEOUT = 20000;      // Stair detection timeout (ms)
unsigned long FINISH_LINE_DEBOUNCE = 50;  // Finish line confirmation delay (ms)

// Sensor Thresholds
#define SENSOR_THRESHOLD_HIGH 512    // Analog threshold (if using analog sensors)
#define SENSOR_DEBOUNCE_TIME 10      // Sensor reading stability time (ms)
```

#### **Advanced Navigation Logic**
```cpp
// Custom Turn Algorithms
typedef enum {
  TURN_SOFT_LEFT = -1,
  TURN_FORWARD = 0, 
  TURN_SOFT_RIGHT = 1,
  TURN_HARD_LEFT = -2,
  TURN_HARD_RIGHT = 2,
  TURN_RECOVERY_LEFT = -3,
  TURN_RECOVERY_RIGHT = 3
} TurnDirection;

// Sensor Fusion Function
TurnDirection calculateOptimalTurn(bool sensors[4]) {
  // Implement weighted sensor algorithm
  int position = (sensors[0] * -3) + (sensors[1] * -1) + 
                 (sensors[2] * 1) + (sensors[3] * 3);
  
  // Convert position to turn command
  if (position < -2) return TURN_HARD_LEFT;
  else if (position < 0) return TURN_SOFT_LEFT;
  else if (position > 2) return TURN_HARD_RIGHT;
  else if (position > 0) return TURN_SOFT_RIGHT;
  else return TURN_FORWARD;
}
```

---

## 🔧 Comprehensive Troubleshooting

### 🎮 RC Car Issues

#### **🔌 Connection Problems**
```bash
# Symptom: Cannot access web interface
Checks:
1. Verify ESP32 power LED is on
2. Check WiFi network "Mat ngu trien mien" appears in available networks
3. Confirm device connected to correct network (not home WiFi)
4. Try different browser or clear browser cache
5. Check Serial Monitor for ESP32 startup messages

# Solution Steps:
- Reset ESP32 and wait 30 seconds for full startup
- Verify AP IP address in Serial Monitor (should be 192.168.4.1)
- Test direct IP access: http://192.168.4.1
- Check firewall settings on connecting device
```

#### **🔧 Motor Control Issues**
```bash
# Symptom: Motors not responding to commands
Hardware Checks:
1. Verify L298N power supply (7-12V for motor power)
2. Check all motor wire connections (no loose wires)
3. Test motor function by applying direct voltage
4. Verify ESP32 5V output powering L298N logic

Software Checks:
1. Monitor WebSocket messages in browser developer console
2. Check Serial Monitor for motor command confirmations
3. Verify GPIO pin definitions match physical connections
4. Test individual motor functions using serial commands
```

#### **🦾 Servo Problems**
```bash
# Symptom: Servos jittery or not moving
Power Issues:
- Ensure adequate power supply (servos draw 100-500mA each)
- Use external 5V regulator for servo power if needed
- Check for voltage drops under load

Signal Issues:
- Verify servo signal wires properly connected
- Check for electromagnetic interference from motors
- Ensure proper grounding between ESP32 and servos
- Test servos individually to isolate problems
```

### 🤖 Self-Driving Car Issues

#### **📍 Navigation Problems**
```bash
# Symptom: Car veers off line consistently
Sensor Calibration:
1. Check sensor height (5-10mm above surface optimal)
2. Verify sensors are perpendicular to line direction
3. Test each sensor individually in Serial Monitor
4. Ensure even lighting without shadows or glare
5. Calibrate sensor spacing for line width

# Symptom: Car overshoots turns
Algorithm Tuning:
- Reduce baseSpeed for better control
- Increase turnSpeed for sharper corrections
- Adjust MIN_TURN_TIME for turn stability
- Fine-tune sensor positions
```

#### **⚡ Power & Performance Issues**
```bash
# Symptom: Inconsistent behavior or random resets
Power Supply:
1. Check battery voltage (should be >7V under load)
2. Verify adequate current capacity (2A+ recommended)
3. Test system with fresh batteries
4. Check for loose power connections

Performance:
1. Monitor Serial output for error messages
2. Check loop timing (should be <50ms per cycle)
3. Verify PWM frequencies don't interfere with each other
4. Test with simplified code to isolate issues
```

#### **🔍 Sensor Debugging**
```bash
# Systematic Sensor Testing:
1. Upload basic sensor reading sketch
2. Move car manually over line while monitoring Serial
3. Verify sensors trigger at correct line positions
4. Check for electrical noise in readings
5. Test different line materials and widths

# Expected Sensor Patterns:
Line Position    | LM | LI | RI | RM | Action
Centered         | 0  | 1  | 1  | 0  | Forward
Slight Left      | 0  | 1  | 0  | 0  | Soft Right
Slight Right     | 0  | 0  | 1  | 0  | Soft Left  
Hard Left        | 1  | 0  | 0  | 0  | Hard Right
Hard Right       | 0  | 0  | 0  | 1  | Hard Left
```

### 🛠️ Library & Compilation Issues

#### **📚 WebServer Conflicts**
The included `LibraryConflictSolver.h` resolves most issues, but if problems persist:

```cpp
// Option 1: Use conflict resolver (recommended)
#include "LibraryConflictSolver.h"
// This must be the FIRST include in your sketch

// Option 2: Manual conflict resolution
#define WEBSERVER_H
#include <WiFi.h>
#include <WebServer.h>
// Include other libraries after defining WEBSERVER_H

// Option 3: Use only one web server library
// Remove either ESPAsyncWebServer OR WebServer, not both
```

#### **🔧 Compilation Errors**
```bash
# Common Error: "HTTP_GET was not declared"
Solution: Ensure LibraryConflictSolver.h is included first

# Common Error: "Class AsyncWebServer not found"  
Solution: Install ESPAsyncWebServer library via Library Manager

# Common Error: "ESP32Servo.h not found"
Solution: Install ESP32Servo library (NOT the standard Servo library)

# Memory Issues: "Sketch too big"
Solution: 
- Select proper ESP32 board with 4MB flash
- Enable minimal SPIFFS if needed
- Remove debug code for production builds
```

---

## 📈 Future Development Roadmap

### 🔮 Planned Enhancements

#### **Phase 1: Core Improvements (Q1 2024)**
- [ ] **Battery Management System**: Real-time voltage monitoring with low-battery warnings
- [ ] **Advanced Servo Control**: Position feedback using potentiometers for precise positioning
- [ ] **Enhanced Web UI**: Dark mode, customizable layouts, touch gesture support
- [ ] **Configuration System**: Web-based parameter adjustment without code modification

#### **Phase 2: Intelligence & Sensors (Q2 2024)**  
- [ ] **Computer Vision**: ESP32-CAM integration for object detection and tracking
- [ ] **Ultrasonic Sensors**: HC-SR04 modules for obstacle avoidance and distance measurement
- [ ] **IMU Integration**: 9-DOF sensors for orientation tracking and stability control
- [ ] **GPS Navigation**: Outdoor autonomous navigation with waypoint following

#### **Phase 3: Communication & Control (Q3 2024)**
- [ ] **Mobile Application**: Native Android/iOS app with enhanced features
- [ ] **Voice Control**: Speech recognition for hands-free operation
- [ ] **Multi-Car Coordination**: Mesh networking for swarm robotics applications
- [ ] **Cloud Integration**: Remote monitoring and control via internet connectivity

#### **Phase 4: Advanced Features (Q4 2024)**
- [ ] **Machine Learning**: On-device learning for improved navigation algorithms
- [ ] **Modular Design**: Plug-and-play sensor and actuator modules
- [ ] **Advanced Arm Control**: Inverse kinematics for precise end-effector positioning
- [ ] **Environmental Mapping**: SLAM (Simultaneous Localization and Mapping) capabilities

### 🤝 Contributing Guidelines

#### **How to Contribute**
1. **Fork the Repository**: Create your own copy for development
2. **Create Feature Branch**: `git checkout -b feature/amazing-feature`
3. **Follow Code Standards**: Use consistent formatting and commenting
4. **Test Thoroughly**: Verify functionality on actual hardware
5. **Document Changes**: Update README and add inline comments
6. **Submit Pull Request**: Provide detailed description of changes

#### **Code Standards**
```cpp
// Use descriptive variable names
int currentServoPosition = 90;  // Good
int pos = 90;                   // Avoid

// Add function documentation
/**
 * @brief Moves servo to target position smoothly
 * @param servoIndex Index of servo (0-5)
 * @param targetAngle Desired angle (0-180)
 * @param speed Movement speed (1-10)
 * @return true if successful, false if invalid parameters
 */
bool moveServoSmoothly(int servoIndex, int targetAngle, int speed);

// Use constants instead of magic numbers
#define MAX_SERVO_ANGLE 180
#define MIN_SERVO_ANGLE 0
#define DEFAULT_SERVO_SPEED 5
```

#### **Testing Requirements**
- [ ] **Hardware Testing**: Verify on actual ESP32 and motor hardware
- [ ] **Edge Cases**: Test boundary conditions and error scenarios  
- [ ] **Performance**: Ensure changes don't impact system responsiveness
- [ ] **Compatibility**: Test with different ESP32 board variants
- [ ] **Documentation**: Update pin diagrams and usage instructions

---

## 📄 License & Legal Information

### **Open Source License**
This project is released under the **MIT License**, which permits:
- ✅ Commercial use
- ✅ Distribution and modification
- ✅ Private use
- ✅ Patent use (where applicable)

**Requirements:**
- 📋 Include original license and copyright notice
- 📋 Document significant changes

### **Third-Party Libraries**
This project uses several open-source libraries, each with their own licenses:
- **ESP32Servo**: MIT License
- **AsyncTCP**: LGPL-3.0 License  
- **ESPAsyncWebServer**: LGPL-3.0 License
- **Arduino ESP32 Core**: LGPL-2.1 License

### **Hardware Considerations**
- This software is designed for educational and hobbyist use
- Users are responsible for safe operation and local regulation compliance
- No warranty provided for commercial or safety-critical applications

---

## 🙋‍♂️ Community & Support

### **Getting Help**

#### **📞 Support Channels**
1. **GitHub Issues**: Report bugs and request features
2. **Discussions**: Ask questions and share projects
3. **Serial Monitor**: First debugging step for hardware issues
4. **Code Comments**: Detailed explanations throughout source code

#### **🔍 Before Asking for Help**
- [ ] Check this README's troubleshooting section
- [ ] Review Serial Monitor output for error messages
- [ ] Verify all hardware connections match pin diagrams
- [ ] Test with minimal code to isolate problems
- [ ] Search existing GitHub issues for similar problems

#### **📝 When Reporting Issues**
Include the following information:
```bash
# System Information
ESP32 Board: [DevKit v1, NodeMCU, etc.]
Arduino IDE Version: [1.8.19, 2.0.x, etc.]
ESP32 Core Version: [2.0.5, etc.]
Libraries Used: [versions]

# Hardware Setup
Motor Driver: [L298N, etc.]
Power Supply: [voltage, current capacity]
Sensors Used: [specific models]

# Problem Description
Expected Behavior: [what should happen]
Actual Behavior: [what actually happens]
Steps to Reproduce: [detailed steps]
Serial Output: [copy relevant output]
```

### **🌟 Community Projects**

#### **Share Your Build**
We love seeing what the community creates! Share your projects:
- Photos and videos of your builds
- Creative modifications and improvements
- New features and sensor integrations
- Educational content and tutorials

#### **Featured Community Builds**
- 🏆 **Autonomous Delivery Robot**: Modified for package delivery
- 🎯 **Target Practice Bot**: Automated ball launcher with camera targeting
- 🏁 **Racing Series**: High-speed line following competitions
- 🎓 **Educational Platform**: University robotics course integration

---

## 🏆 Acknowledgments

### **Special Thanks**
- **ESP32 Community**: For extensive documentation and support
- **Arduino Project**: For the incredible development platform
- **Open Source Contributors**: Libraries that make this project possible
- **Beta Testers**: Community members who tested and provided feedback

### **Inspiration & References**
- Academic robotics research papers
- Commercial robotics platforms
- Community robotics projects
- Educational robotics curricula

---

**🚗 Happy Building and Happy Coding! 🤖**

*"The best way to learn robotics is to build robots!"*

---

*Last Updated: January 2024 | Version 2.0*
*For the latest updates, visit: [GitHub Repository](https://github.com/yourusername/Robocar-Microcontroller)*
- **Jumper wires and breadboard**

### RC Car Specific
- **6x Servo Motors** (SG90 or similar)
- **Robotic arm assembly**
- **LED indicator** (optional)

### Self-Driving Car Specific
- **4x Digital Line Sensors** (TCRT5000 or similar)
- **1x Stair Detection Sensor**
- **Sensor mounting bracket**

---

## 📌 Pin Configuration

### 🎮 RC Car Pin Layout

#### Motor Driver (L298N)
| Pin | ESP32 | Function |
|-----|-------|----------|
| ENA | 18    | Right motor PWM |
| IN1 | 19    | Right motor direction 1 |
| IN2 | 21    | Right motor direction 2 |
| ENB | 15    | Left motor PWM |
| IN3 | 4     | Left motor direction 1 |
| IN4 | 2     | Left motor direction 2 |

#### Servo Motors
| Servo | ESP32 Pin | Function |
|-------|-----------|----------|
| Base | 27 | Base rotation |
| Shoulder | 26 | Shoulder movement |
| Elbow | 25 | Elbow joint |
| Gripper | 33 | Gripper control |
| Servo5 | 32 | Auxiliary servo 1 |
| Servo6 | 21 | Auxiliary servo 2 |

#### Indicators
| Component | ESP32 Pin | Function |
|-----------|-----------|----------|
| Status LED | T2 | Connection status |

### 🤖 Self-Driving Car Pin Layout

#### Motor Driver (L298N)
| Pin | ESP32 | Function |
|-----|-------|----------|
| ENA | 5     | Right motor PWM |
| ENB | 18    | Left motor PWM |
| IN1 | 2     | Right motor direction 1 |
| IN2 | 4     | Right motor direction 2 |
| IN3 | 16    | Left motor direction 1 |
| IN4 | 17    | Left motor direction 2 |

#### Line Sensors
| Sensor | ESP32 Pin | Function |
|--------|-----------|----------|
| LEFT_MOST | 13 | Leftmost line sensor |
| LEFT_INNER | 12 | Left inner sensor |
| RIGHT_INNER | 14 | Right inner sensor |
| RIGHT_MOST | 27 | Rightmost sensor |
| STAIR_SENSOR | 26 | Stair detection |

---

## ⚡ Features

### 🎮 RC Car Features
- **Web-based Control**: Responsive HTML interface accessible via WiFi
- **Real-time Communication**: WebSocket for instant response
- **Robotic Arm Control**: 6-axis servo control with smooth movements
- **Mobile Compatible**: Touch-friendly interface for smartphones
- **Auto-shoot Function**: Automated ball shooting sequence
- **Status Indicators**: LED feedback for connection status

### 🤖 Self-Driving Car Features
- **Line Following**: Advanced algorithm with multiple sensor inputs
- **Intelligent Turning**: Adaptive turning based on line detection
- **Stair Detection**: Safety feature to prevent falls
- **Speed Control**: Multiple speed settings for different scenarios
- **Debug Mode**: Serial output for troubleshooting
- **Recovery System**: Handles lost line situations

---

## 🚀 Getting Started

### 1. **Hardware Setup**
1. Assemble your chassis and mount the ESP32
2. Connect motors to L298N driver
3. Wire the ESP32 according to pin configuration
4. For RC car: Mount and connect servo motors
5. For self-driving: Mount and align line sensors

### 2. **Software Installation**
1. Install Arduino IDE with ESP32 board support
2. Install required libraries:
   ```
   - ESP32Servo
   - AsyncTCP
   - ESPAsyncWebServer
   - WiFi
   - WebSocketsServer
   ```

### 3. **Upload Code**
1. Open the appropriate `.ino` file
2. Select your ESP32 board in Arduino IDE
3. Upload the code to your ESP32

### 4. **Configuration**

#### RC Car Setup
1. Connect to WiFi network "Mat ngu trien mien"
2. Password: "monggiadinhanlanh"
3. Open browser and go to `192.168.4.1`
4. Use the web interface to control the car

#### Self-Driving Car Setup
1. Place car on a line track
2. Ensure sensors are properly aligned
3. Open Serial Monitor for debugging
4. Adjust speeds and thresholds as needed

---

## 🎮 Usage Instructions

### RC Car Control

#### **Web Interface**
- **Servo Control**: Use sliders to adjust arm position (0-180°)
- **Car Movement**: Touch arrow buttons for directional control
- **Shoot Function**: Press "SHOOT BALL" for automated sequence

#### **WebSocket Commands**
```javascript
// Movement commands
"Forward,1"   // Move forward
"Backward,1"  // Move backward
"Left,1"      // Turn left
"Right,1"     // Turn right

// Servo commands
"Base,90"     // Set base servo to 90°
"Gripper,45"  // Set gripper to 45°
```

### Self-Driving Car Control

#### **Serial Commands**
```
timeout:30    // Set stair detection timeout to 30 seconds
```

#### **Sensor Reading**
- **Black line**: Sensor reads `1`
- **White surface**: Sensor reads `0`
- **All sensors black**: Finish line detected
- **All sensors white**: Line lost - recovery mode

---

## ⚙️ Configuration

### RC Car Settings
```cpp
// WiFi Configuration
const char* ap_ssid = "Mat ngu trien mien";
const char* ap_password = "monggiadinhanlanh";

// Motor Speeds
#define MOTOR_BASE_SPEED 250
#define MOTOR_SLOW_SPEED 160
```

### Self-Driving Car Settings
```cpp
// Motor Speeds
int baseSpeed = 255;        // Normal forward speed
int turnSpeed = 190;        // Inner wheel speed in turns
int hardTurnSpeed = 80;     // Hard turn speed

// Timing
unsigned long stairTimeoutDuration = 20000; // 20 seconds
```

---

## 🔧 Troubleshooting

### Common Issues

#### **RC Car**
- **Can't connect to web interface**: Check WiFi credentials and IP address
- **Motors not responding**: Verify L298N connections and power supply
- **Servo jittery movement**: Check power supply capacity

#### **Self-Driving Car**
- **Car veers off line**: Calibrate sensor positions and thresholds
- **Doesn't detect finish line**: Ensure all sensors are working
- **Stuck at stairs**: Adjust stair sensor sensitivity

### **Library Conflicts**
If you encounter compilation errors, use the provided `LibraryConflictSolver.h`:
```cpp
#include "LibraryConflictSolver.h"
```

---

## 📈 Future Enhancements

### Planned Features
- [ ] **Camera Integration**: Live video streaming
- [ ] **Voice Control**: Speech recognition commands
- [ ] **Obstacle Avoidance**: Ultrasonic sensor integration
- [ ] **Path Planning**: GPS-based navigation
- [ ] **Mobile App**: Dedicated smartphone application
- [ ] **Battery Monitoring**: Real-time power level display

### Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

---

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

---

## 🙋‍♂️ Support

For questions and support:
- Open an issue on GitHub
- Check the troubleshooting section
- Review the code comments for detailed explanations

---

**Happy Building! 🚗🤖**
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include "index.h"

#define CMD_STOP 0
#define CMD_FORWARD 1
#define CMD_BACKWARD 2
#define CMD_LEFT 4
#define CMD_RIGHT 8

// Add slow movement commands
#define CMD_SLOW_FORWARD 101
#define CMD_SLOW_BACKWARD 102
#define CMD_SLOW_LEFT 104
#define CMD_SLOW_RIGHT 108

// Servo commands
#define CMD_SERVO1 16
#define CMD_SERVO2 32
#define CMD_SERVO3 64

// Motor pin definitions - corrected to match standard L298N wiring
// Right motor
#define ENA_PIN 18  // Enable pin for right motor
#define IN1_PIN 19  // Control pin 1 for right motor
#define IN2_PIN 21  // Control pin 2 for right motor
// Left motor
#define ENB_PIN 15   // Enable pin for left motor - using pin 5 which supports PWM
#define IN3_PIN 4  // Control pin 1 for left motor - changed to 15 (supports output)
#define IN4_PIN 2   // Control pin 2 for left motor - changed to 2 (supports output)

// LED indicator pin
#define LED_PIN T2  // Using T2 pin for connection status LED

// LED status variables
bool clientConnected = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Servo pins - modified to avoid conflict with motor pins
#define SERVO1_PIN 27
#define SERVO2_PIN 26
#define SERVO3_PIN 33

// Motor speeds
#define MOTOR_BASE_SPEED 250
#define MOTOR_SLOW_SPEED 160

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Servo positions (0-180)
int servo1Pos = 90;
int servo2Pos = 90;
int servo3Pos = 50;

// Last servo positions to track changes
int lastServo1Pos = 90;
int lastServo2Pos = 90;
int lastServo3Pos = 50;

// Access point credentials
const char* ap_ssid = "Mat ngu trien mien";     // Name of the access point
const char* ap_password = "monggiadinhanlanh";     // Password for the access point (min 8 chars)

// IP configurations for AP mode
IPAddress ap_local_IP(192, 168, 4, 1);
IPAddress ap_gateway(192, 168, 4, 1);
IPAddress ap_subnet(255, 255, 255, 0);

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket server on port 81

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      // Update connection status when all clients are disconnected
      if (webSocket.connectedClients() == 0) {
        clientConnected = false;
      }
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        clientConnected = true;
      }
      break;
    case WStype_TEXT:
      {
        String message = String((char*)payload);
        Serial.print("Received message: ");
        Serial.println(message);
        
        // Check if it's a servo position command (format: "CMD_SERVO#:position")
        if (message.indexOf(":") > 0) {
          int cmdPos = message.indexOf(":");
          int cmd = message.substring(0, cmdPos).toInt();
          int pos = message.substring(cmdPos + 1).toInt();
          
          // Process servo commands
          switch (cmd) {
            case CMD_SERVO1:
              servo1Pos = pos;
              servo1.write(servo1Pos);
              Serial.printf("Setting Servo 1 to %d degrees\n", servo1Pos);
              break;
            case CMD_SERVO2:
              servo2Pos = pos;
              servo2.write(servo2Pos);
              Serial.printf("Setting Servo 2 to %d degrees\n", servo2Pos);
              break;
            case CMD_SERVO3:
              servo3Pos = pos;
              servo3.write(servo3Pos);
              Serial.printf("Setting Servo 3 to %d degrees\n", servo3Pos);
              break;
          }
        } 
        else {
          // Regular movement commands
          int command = message.toInt();
          Serial.print("command: ");
          Serial.println(command);

          switch (command) {
            case CMD_STOP:
              Serial.println("Stop");
              CAR_stop();
              break;
            case CMD_FORWARD:
              Serial.println("Move Forward");
              CAR_moveForward();
              break;
            case CMD_BACKWARD:
              Serial.println("Move Backward");
              CAR_moveBackward();
              break;
            case CMD_LEFT:
              Serial.println("Turn Left");
              CAR_turnLeft();
              break;
            case CMD_RIGHT:
              Serial.println("Turn Right");
              CAR_turnRight();
              break;
            case CMD_SLOW_FORWARD:
              Serial.println("Move Forward Slowly");
              CAR_moveForwardSlow();
              break;
            case CMD_SLOW_BACKWARD:
              Serial.println("Move Backward Slowly");
              CAR_moveBackwardSlow();
              break;
            case CMD_SLOW_LEFT:
              Serial.println("Turn Left Slowly");
              CAR_turnLeftSlow();
              break;
            case CMD_SLOW_RIGHT:
              Serial.println("Turn Right Slowly");
              CAR_turnRightSlow();
              break;
            default:
              Serial.println("Unknown command");
          }
        }
      }
      break;
  }
}

void handleRoot() {
  Serial.println("Web Server: received a web page request");
  server.send(200, "text/html", HTML_CONTENT);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 RC Car starting in AP mode...");

  // Configure motor pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  
  // Initialize motor pins to LOW
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);

  // Initialize ESP32 Servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach servos to pins and configure
  servo1.setPeriodHertz(50);      // Standard 50Hz servo
  servo1.attach(SERVO1_PIN, 544, 2400); // Adjust min/max pulse width if needed
  servo1.write(servo1Pos);         // Set to initial position
  
  servo2.setPeriodHertz(50);
  servo2.attach(SERVO2_PIN, 544, 2400);
  servo2.write(servo2Pos);
  
  servo3.setPeriodHertz(50);
  servo3.attach(SERVO3_PIN, 544, 2400);
  servo3.write(servo3Pos);
  
  Serial.println("Servos initialized");

  // Set WiFi to AP mode only
  WiFi.mode(WIFI_AP);

  // Configure and start AP
  configureWiFiAP();

  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");

  // Set up web server routes
  server.on("/", HTTP_GET, handleRoot);
  
  // Start server
  server.begin();
  Serial.println("HTTP server started on port 80");

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED off
  Serial.println("LED indicator initialized on pin T2");
}

void configureWiFiAP() {
  // Configure AP with static IP
  WiFi.softAPConfig(ap_local_IP, ap_gateway, ap_subnet);
  
  // Create WiFi Access Point
  Serial.println("Creating Access Point...");
  bool apSuccess = WiFi.softAP(ap_ssid, ap_password);
  
  if (apSuccess) {
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("AP SSID: ");
    Serial.println(ap_ssid);
    Serial.print("AP Password: ");
    #ifdef DEBUG
        Serial.println(ap_password);
    #endif
  } else {
    Serial.println("Failed to create Access Point!");
  }
}

void loop() {
  server.handleClient();  // Handle HTTP requests
  webSocket.loop();       // Handle WebSocket requests

  // LED indicator logic
  if (!clientConnected) {
    // Blink LED at 1 second interval when waiting for connection
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= 1000) {
      lastBlinkTime = currentTime;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      
      // Print periodic debug info
      if (ledState) {
        Serial.println("Waiting for WebSocket connection...");
        Serial.print("Access point IP: ");
        Serial.println(WiFi.softAPIP());
        Serial.print("Connected clients: ");
        Serial.println(WiFi.softAPgetStationNum());
      }
    }
  } else {
    // Keep LED on when client is connected
    digitalWrite(LED_PIN, HIGH);
  }
}

void CAR_moveForward() {
  // First set direction pins
  digitalWrite(IN1_PIN, HIGH);  // Right motor forward
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);  // Left motor forward
  digitalWrite(IN4_PIN, LOW);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_BASE_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_BASE_SPEED);  // Left motor speed
  
  Serial.println("Moving forward: Right(IN1 HIGH), Left(IN3 HIGH)");
  
  // Debug motor signals
  Serial.print("Motor pins - IN1: ");
  Serial.print(digitalRead(IN1_PIN));
  Serial.print(", IN2: ");
  Serial.print(digitalRead(IN2_PIN));
  Serial.print(", IN3: ");
  Serial.print(digitalRead(IN3_PIN));
  Serial.print(", IN4: ");
  Serial.println(digitalRead(IN4_PIN));
}

void CAR_moveBackward() {
  // First set direction pins
  digitalWrite(IN1_PIN, LOW);   // Right motor backward
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);   // Left motor backward
  digitalWrite(IN4_PIN, HIGH);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_BASE_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_BASE_SPEED);  // Left motor speed
  
  Serial.println("Moving backward: Right(IN2 HIGH), Left(IN4 HIGH)");
  
  // Debug motor signals
  Serial.print("Motor pins - IN1: ");
  Serial.print(digitalRead(IN1_PIN));
  Serial.print(", IN2: ");
  Serial.print(digitalRead(IN2_PIN));
  Serial.print(", IN3: ");
  Serial.print(digitalRead(IN3_PIN));
  Serial.print(", IN4: ");
  Serial.println(digitalRead(IN4_PIN));
}

void CAR_turnLeft() {
  // First set direction pins
  digitalWrite(IN1_PIN, HIGH);  // Right motor forward
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);   // Left motor backward
  digitalWrite(IN4_PIN, HIGH);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_BASE_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_BASE_SPEED);  // Left motor speed
  
  Serial.println("Turning left: Right forward, Left backward");
  
  // Debug motor signals
  Serial.print("Motor pins - IN1: ");
  Serial.print(digitalRead(IN1_PIN));
  Serial.print(", IN2: ");
  Serial.print(digitalRead(IN2_PIN));
  Serial.print(", IN3: ");
  Serial.print(digitalRead(IN3_PIN));
  Serial.print(", IN4: ");
  Serial.println(digitalRead(IN4_PIN));
}

void CAR_turnRight() {
  // First set direction pins
  digitalWrite(IN1_PIN, LOW);   // Right motor backward
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);  // Left motor forward
  digitalWrite(IN4_PIN, LOW);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_BASE_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_BASE_SPEED);  // Left motor speed
  
  Serial.println("Turning right: Right backward, Left forward");
  
  // Debug motor signals
  Serial.print("Motor pins - IN1: ");
  Serial.print(digitalRead(IN1_PIN));
  Serial.print(", IN2: ");
  Serial.print(digitalRead(IN2_PIN));
  Serial.print(", IN3: ");
  Serial.print(digitalRead(IN3_PIN));
  Serial.print(", IN4: ");
  Serial.println(digitalRead(IN4_PIN));
}

void CAR_stop() {
  // First set direction pins to LOW
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  
  // Then set speed to 0
  analogWrite(ENA_PIN, 0);  // Right motor speed
  analogWrite(ENB_PIN, 0);  // Left motor speed
  
  Serial.println("Stopping: All direction pins LOW");
}

// New slow movement functions
void CAR_moveForwardSlow() {
  // First set direction pins
  digitalWrite(IN1_PIN, HIGH);  // Right motor forward
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);  // Left motor forward
  digitalWrite(IN4_PIN, LOW);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);  // Left motor speed
  
  Serial.println("Moving forward slowly");
}

void CAR_moveBackwardSlow() {
  // First set direction pins
  digitalWrite(IN1_PIN, LOW);   // Right motor backward
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);   // Left motor backward
  digitalWrite(IN4_PIN, HIGH);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);  // Left motor speed
  
  Serial.println("Moving backward slowly");
}

void CAR_turnLeftSlow() {
  // First set direction pins
  digitalWrite(IN1_PIN, HIGH);  // Right motor forward
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);   // Left motor backward
  digitalWrite(IN4_PIN, HIGH);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);  // Left motor speed
  
  Serial.println("Turning left slowly");
}

void CAR_turnRightSlow() {
  // First set direction pins
  digitalWrite(IN1_PIN, LOW);   // Right motor backward
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);  // Left motor forward
  digitalWrite(IN4_PIN, LOW);
  
  // Then set speed
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);  // Right motor speed
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);  // Left motor speed
  
  Serial.println("Turning right slowly");
}

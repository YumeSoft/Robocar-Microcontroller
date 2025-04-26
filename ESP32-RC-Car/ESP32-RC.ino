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

// Servo commands
#define CMD_SERVO1 16
#define CMD_SERVO2 32
#define CMD_SERVO3 64
<<<<<<< HEAD
#define ENA_PIN 16  // Motor driver ENA pin
#define IN1_PIN 17  // Motor driver IN1 pin
#define IN2_PIN 5   // Motor driver IN2 pin
#define IN3_PIN 18  // Motor driver IN3 pin
#define IN4_PIN 19  // Motor driver IN4 pin
#define ENB_PIN 21  // Motor driver ENB pin

// LED indicator pin
#define LED_PIN T2  // Using T2 pin for connection status LED

// LED status variables
bool clientConnected = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;
=======
#define CMD_SERVO4 128

#define ENA_PIN 15  // The ESP32 pin GPIO14 connected to the ENA pin L298N
#define IN1_PIN 2   // The ESP32 pin GPIO27 connected to the IN1 pin L298N
#define IN2_PIN 4   // The ESP32 pin GPIO26 connected to the IN2 pin L298N
#define IN3_PIN 16  // The ESP32 pin GPIO25 connected to the IN3 pin L298N
#define IN4_PIN 17  // The ESP32 pin GPIO33 connected to the IN4 pin L298N
#define ENB_PIN 5   // The ESP32 pin GPIO32 connected to the ENB pin L298N
>>>>>>> parent of 08cad5e (Enhance RC car control with slow movement commands and improved web interface styling)

// Servo pins - modified to avoid conflict with motor pins
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 27

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Servo positions (0-180)
int servo1Pos = 90;
int servo2Pos = 90;
int servo3Pos = 90;
int servo4Pos = 90;

// Access point credentials
const char* ap_ssid = "ESP32_RC_Car";     // Name of the access point
const char* ap_password = "12345678";     // Password for the access point (min 8 chars)

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
              if (pos != servo1Pos) { // Only update if position changed
                servo1Pos = pos;
                servo1.write(servo1Pos);
                Serial.printf("Setting Servo 1 to %d degrees\n", servo1Pos);
              }
              break;
            case CMD_SERVO2:
              if (pos != servo2Pos) { // Only update if position changed
                servo2Pos = pos;
                servo2.write(servo2Pos);
                Serial.printf("Setting Servo 2 to %d degrees\n", servo2Pos);
              }
              break;
            case CMD_SERVO3:
              if (pos != servo3Pos) { // Only update if position changed
                servo3Pos = pos;
                servo3.write(servo3Pos);
                Serial.printf("Setting Servo 3 to %d degrees\n", servo3Pos);
              }
              break;
            case CMD_SERVO4:
              servo4Pos = pos;
              servo4.write(servo4Pos);
              Serial.printf("Setting Servo 4 to %d degrees\n", servo4Pos);
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
  digitalWrite(ENA_PIN, MOTOR_BASE_SPEED);  // set full speed
  digitalWrite(ENB_PIN, MOTOR_BASE_SPEED);  // set full speed

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
  
  servo4.setPeriodHertz(50);
  servo4.attach(SERVO4_PIN, 500, 2400);
  servo4.write(servo4Pos);
  
  Serial.println("Servos initialized");

  // Set WiFi to AP mode only
  WiFi.mode(WIFI_AP);

  // Configure and start AP
  configureWiFiAP();

  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Set up web server routes
  server.on("/", HTTP_GET, handleRoot);
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");

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
    }
  } else {
    // Keep LED on when client is connected
    digitalWrite(LED_PIN, HIGH);
  }
}

void CAR_moveForward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_moveBackward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void CAR_turnLeft() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_turnRight() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_stop() {
  // First set the direction pins to LOW
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  
  // Then reduce the speed to 0 to prevent any residual current
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

<<<<<<< HEAD
// New slow movement functions
void CAR_moveForwardSlow() {
  // First set speed to avoid current spikes
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);
  
  // Small delay to allow PWM to stabilize
  delay(5);
  
  // Then set direction
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_moveBackwardSlow() {
  // First set speed to avoid current spikes
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);
  
  // Small delay to allow PWM to stabilize
  delay(5);
  
  // Then set direction
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void CAR_turnLeftSlow() {
  // First set speed to avoid current spikes
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);
  
  // Small delay to allow PWM to stabilize
  delay(5);
  
  // Then set direction - only activate necessary motors
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void CAR_turnRightSlow() {
  // First set speed to avoid current spikes
  analogWrite(ENA_PIN, MOTOR_SLOW_SPEED);
  analogWrite(ENB_PIN, MOTOR_SLOW_SPEED);
  
  // Small delay to allow PWM to stabilize
  delay(5);
  
  // Then set direction - only activate necessary motors
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}
=======
void moveServo1() {
  servo1.write(servo1Pos);
}

void moveServo2() {
  servo2.write(servo2Pos);
}

void moveServo3() {
  servo3.write(servo3Pos);
}

void moveServo4() {
  servo4.write(servo4Pos);
}
>>>>>>> parent of 08cad5e (Enhance RC car control with slow movement commands and improved web interface styling)

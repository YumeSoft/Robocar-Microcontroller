#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "index.h"
#include <esp_wifi.h>
#include <lwip/dns.h>

#define CMD_STOP 0
#define CMD_FORWARD 1
#define CMD_BACKWARD 2
#define CMD_LEFT 4
#define CMD_RIGHT 8

#define ENA_PIN 14  // The ESP32 pin GPIO14 connected to the ENA pin L298N
#define IN1_PIN 27  // The ESP32 pin GPIO27 connected to the IN1 pin L298N
#define IN2_PIN 26  // The ESP32 pin GPIO26 connected to the IN2 pin L298N
#define IN3_PIN 25  // The ESP32 pin GPIO25 connected to the IN3 pin L298N
#define IN4_PIN 33  // The ESP32 pin GPIO33 connected to the IN4 pin L298N
#define ENB_PIN 32  // The ESP32 pin GPIO32 connected to the ENB pin L298N

// Access point credentials
const char* ap_ssid = "ESP32_RC_Car";     // Name of the access point
const char* ap_password = "12345678";     // Password for the access point (min 8 chars)

// Home network credentials - replace with your own
const char* sta_ssid = "Khong biet 5G";     // Name of your home WiFi network
const char* sta_password = "hoilamchithe"; // Password for your home WiFi

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
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      String angle = String((char*)payload);
      int command = angle.toInt();
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
      break;
  }
}

void handleRoot() {
  Serial.println("Web Server: received a web page request");
  server.send(200, "text/html", HTML_CONTENT);
}

void setup() {
  Serial.begin(9600);
  Serial.println("ESP32 RC Car with dual WiFi mode starting...");

  // Configure motor pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);  // set full speed
  digitalWrite(ENB_PIN, HIGH);  // set full speed

  // Set WiFi to dual mode
  WiFi.mode(WIFI_AP_STA);

  // Configure AP mode
  configureWiFiAP();
  
  // Connect to home WiFi
  configureWiFiStation();
  
  // Setup NAT/IP forwarding if station connected
  if (WiFi.status() == WL_CONNECTED) {
    setupNAT();
  }

  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Set up web server routes
  server.on("/", HTTP_GET, handleRoot);
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
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
  } else {
    Serial.println("Failed to create Access Point!");
  }
}

void configureWiFiStation() {
  Serial.print("Connecting to home WiFi network: ");
  Serial.println(sta_ssid);
  
  WiFi.begin(sta_ssid, sta_password);
  
  // Try to connect for 20 seconds
  // int attempts = 0;
  // while (WiFi.status() != WL_CONNECTED && attempts < 5) {
  //   delay(1000);
  //   Serial.print(".");
  //   attempts++;
  // }
  WiFi.mode(WIFI_AP);

  // if (WiFi.status() == WL_CONNECTED) {
  //   Serial.println("");
  //   Serial.println("Successfully connected to home WiFi");
  //   Serial.print("STA IP address: ");
  //   Serial.println(WiFi.localIP());
  // } else {
  //   Serial.println("");
  //   Serial.println("Failed to connect to home WiFi, continuing in AP-only mode");
  //   // Disable station mode if it failed to connect
  //   WiFi.mode(WIFI_AP);
  // }
}

void setupNAT() {
  // Disable WiFi power save mode for better performance
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  Serial.println("NAT router enabled");
  Serial.println("Devices connected to the ESP32 AP can now access the internet");
}

void loop() {
  server.handleClient();  // Handle HTTP requests
  webSocket.loop();       // Handle WebSocket requests
  
  // Check WiFi station status periodically
  static unsigned long lastWiFiCheck = 0;
  const unsigned long CHECK_INTERVAL = 30000; // 30 seconds
  
  if (millis() - lastWiFiCheck > CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi still connected to: " + String(sta_ssid));
    } else if (WiFi.getMode() & WIFI_STA) {
      Serial.println("WiFi connection lost, attempting to reconnect...");
      WiFi.disconnect();
      WiFi.begin(sta_ssid, sta_password);
    }
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
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}
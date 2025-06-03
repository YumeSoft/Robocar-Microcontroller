#include <Arduino.h>          // Thư viện cốt lõi của Arduino
#include <WiFi.h>             // Thư viện WiFi cho ESP32
#include <AsyncTCP.h>         // Thư viện TCP  
#include <ESPAsyncWebServer.h> // Thư viện Web Server 
#include <ESP32Servo.h>       // Thư viện điều khiển Servo cho ESP32
#include <iostream>           // Thư viện input/output tiêu chuẩn C++
#include <sstream>            // Thư viện thao tác với chuỗi

// Cấu hình Servo
struct ServoPins {
  Servo servo;                // Đối tượng Servo
  int servoPin;               // Chân kết nối Servo
  String servoName;           // Tên Servo (để nhận diện)
  int initialPosition;        // Vị trí ban đầu của Servo (0-180 độ)
};

// Danh sách các Servo được sử dụng
std::vector<ServoPins> servoPins = {
  { Servo(), 27, "Base", 90 },     // Servo 1: Base, chân 27, vị trí ban đầu 90°
  { Servo(), 26, "Shoulder", 90 }, // Servo 2: Shoulder, chân 26, vị trí ban đầu 90°
  { Servo(), 25, "Elbow", 90 },    // Servo 3: Elbow, chân 25, vị trí ban đầu 90°
  { Servo(), 33, "Gripper", 90 },  // Servo 4: Gripper, chân 33, vị trí ban đầu 90°
  { Servo(), 32, "Servo5", 0 },    // Servo 5: Servo phụ 1, chân 32, vị trí ban đầu 0°
  { Servo(), 21, "Servo6", 0 }     // Servo 6: Servo phụ 2, chân 21, vị trí ban đầu 0°
};

// Cấu hình Motor
#define MOTOR_A_IN1 5        // Chân IN1 của Motor A
#define MOTOR_A_IN2 4        // Chân IN2 của Motor A
#define MOTOR_B_IN1 2        // Chân IN1 của Motor B
#define MOTOR_B_IN2 15       // Chân IN2 của Motor B

// Cấu hình WiFi
const char* ssid = "Name";           // Tên WiFi IP
const char* password = "pass";    // Mật khẩu WiFi IP
IPAddress local_IP(192, 168, 1, 12);  // Địa chỉ IP tĩnh
IPAddress gateway(192, 168, 1, 12);   // Gateway
IPAddress subnet(255, 255, 255, 0);   // Subnet mask

// Khởi tạo server
AsyncWebServer server(80);                          // Web server chạy trên cổng 80
AsyncWebSocket wsRobotArmInput("/RobotArmInput");   // WebSocket để nhận lệnh điều khiển

// Biến điều khiển xe
struct ControlState {
  bool forward = false;     // Trạng thái tiến
  bool backward = false;    // Trạng thái lùi
  bool left = false;        // Trạng thái rẽ trái
  bool right = false;       // Trạng thái rẽ phải
} controlState;

// Biến điều khiển Servo
struct ServoControl {
  bool isRotating = false;          // Đang quay Servo hay không
  bool shouldRotate = false;        // Có cần quay Servo không
  unsigned long previousMillis = 0; // Thời điểm lần trước cập nhật
  const long interval = 5;          // Khoảng thời gian giữa các lần cập nhật (ms)
  int currentAngle5 = 0;            // Góc hiện tại của Servo 5
  int currentAngle6 = 0;            // Góc hiện tại của Servo 6
  int targetAngle5 = 0;             // Góc đích của Servo 5
  int targetAngle6 = 0;             // Góc đích của Servo 6
  const int stepSize = 3;           // Bước thay đổi góc mỗi lần cập nhật
  bool rotationState = false;       // Trạng thái quay (để điều khiển Servo 5 và 6)
  unsigned long servo6StartTime = 0; // Thời điểm bắt đầu quay Servo 6
  const int servo6Delay = 1000;     // Độ trễ trước khi quay Servo 6 (ms)
} servoControl;

// Trang HTML với giao diện điều khiển
const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <style>
    :root {
      --primary-color: #FF8C00;
      --secondary-color: #FFA500;
      --accent-color: #FF6347;
      --dark-color: #333;
      --light-color: #FFF8DC;
      --slider-thumb: #FF4500;
      --slider-track: #FFD700;
    }
    
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: var(--light-color);
      margin: 0;
      padding: 10px;
    }
    
    .noselect {
      -webkit-touch-callout: none;
      -webkit-user-select: none;
      -khtml-user-select: none;
      -moz-user-select: none;
      -ms-user-select: none;
      user-select: none;
    }
    
    .header {
      text-align: center;
      margin-bottom: 15px;
      color: var(--dark-color);
    }
    
    .header h1 {
      font-size: 2rem;
      margin: 5px 0;
      color: var(--primary-color);
      text-shadow: 1px 1px 2px rgba(0,0,0,0.1);
    }
    
    .control-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      max-width: 600px;
      margin: 0 auto;
    }
    
    .control-panel {
      background-color: white;
      border-radius: 10px;
      padding: 15px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.1);
    }
    
    .slider-container {
      margin-bottom: 15px;
    }
    
    .slider-label {
      display: flex;
      justify-content: space-between;
      margin-bottom: 5px;
      font-weight: bold;
      color: var(--dark-color);
      font-size: 14px;
    }
    
    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 10px;
      border-radius: 5px;
      background: var(--slider-track);
      outline: none;
    }
    
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: var(--slider-thumb);
      cursor: pointer;
      transition: all 0.2s;
    }
    
    .slider::-moz-range-thumb {
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: var(--slider-thumb);
      cursor: pointer;
    }
    
    .arrow-button {
      background-color: var(--primary-color);
      color: white;
      border-radius: 10px;
      width: 70px;
      height: 70px;
      font-size: 25px;
      border: none;
      margin: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      cursor: pointer;
      transition: all 0.2s;
    }
    
    .arrow-button:hover {
      background-color: var(--accent-color);
      transform: scale(1.05);
    }
    
    .arrow-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 5px;
      width: 220px;
      margin: 0 auto;
    }
    
    .up { grid-column: 2; grid-row: 1; }
    .left { grid-column: 1; grid-row: 2; }
    .right { grid-column: 3; grid-row: 2; }
    .down { grid-column: 2; grid-row: 3; }
    
    .shoot-btn {
      background-color: var(--accent-color);
      color: white;
      border: none;
      border-radius: 5px;
      padding: 10px;
      width: 100%;
      font-weight: bold;
      margin-top: 10px;
      cursor: pointer;
      transition: all 0.2s;
    }
    
    .shoot-btn:hover {
      background-color: #FF4500;
    }
    
    @media (max-width: 600px) {
      .control-grid {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body class="noselect">
  <div class="header">
    <h1>ROBOCAR 2025</h1>
  </div>
  
  <div class="control-grid">
    <div class="control-panel">
      <div class="slider-container">
        <div class="slider-label">
          <span>Gripper</span>
          <span id="gripperValue">90</span>
        </div>
        <input type="range" min="0" max="180" value="90" class="slider" id="Gripper" 
              oninput='document.getElementById("gripperValue").innerText=this.value+"°";sendButtonInput("Gripper",this.value)'>
      </div>
      
      <div class="slider-container">
        <div class="slider-label">
          <span>Elbow</span>
          <span id="elbowValue">90</span>
        </div>
        <input type="range" min="0" max="180" value="90" class="slider" id="Elbow" 
              oninput='document.getElementById("elbowValue").innerText=this.value+"°";sendButtonInput("Elbow",this.value)'>
      </div>
      
      <div class="slider-container">
        <div class="slider-label">
          <span>Shoulder</span>
          <span id="shoulderValue">90</span>
        </div>
        <input type="range" min="0" max="180" value="90" class="slider" id="Shoulder" 
              oninput='document.getElementById("shoulderValue").innerText=this.value+"°";sendButtonInput("Shoulder",this.value)'>
      </div>
      
      <div class="slider-container">
        <div class="slider-label">
          <span>Base</span>
          <span id="baseValue">90</span>
        </div>
        <input type="range" min="0" max="180" value="90" class="slider" id="Base" 
              oninput='document.getElementById("baseValue").innerText=this.value+"°";sendButtonInput("Base",this.value)'>
      </div>
      
      <button id="RotateServos" class="shoot-btn" ontouchend='onclickButton(this)'>SHOOT BALL</button>
    </div>
    
    <div class="control-panel" style="display:flex;flex-direction:column;align-items:center;">
      <div style="margin-bottom:15px;font-weight:bold;color:var(--dark-color);">CAR CONTROL</div>
      <div class="arrow-grid">
        <button class="arrow-button up" 
                ontouchstart='sendButtonInput("Forward","1")' 
                ontouchend='sendButtonInput("Forward","0")'>Go</button>
        <button class="arrow-button left" 
                ontouchstart='sendButtonInput("Left","1")' 
                ontouchend='sendButtonInput("Left","0")'>Left</button>
        <button class="arrow-button right" 
                ontouchstart='sendButtonInput("Right","1")' 
                ontouchend='sendButtonInput("Right","0")'>Right</button>
        <button class="arrow-button down" 
                ontouchstart='sendButtonInput("Backward","1")' 
                ontouchend='sendButtonInput("Backward","0")'>Back</button>
      </div>
    </div>
  </div>

  <script>
    var webSocketRobotArmInputUrl = "ws:\/\/" + window.location.hostname + "/RobotArmInput";
    var websocketRobotArmInput;
    
    function initRobotArmInputWebSocket() {
      websocketRobotArmInput = new WebSocket(webSocketRobotArmInputUrl);
      websocketRobotArmInput.onclose = function() {
        setTimeout(initRobotArmInputWebSocket, 2000);
      };
    }
    
    function sendButtonInput(key, value) {
      var data = key + "," + value;
      if (websocketRobotArmInput.readyState === WebSocket.OPEN) {
        websocketRobotArmInput.send(data);
      }
    }
    
    function onclickButton(button) {
      sendButtonInput(button.id, "1");
    }
    
    window.onload = function() {
      initRobotArmInputWebSocket();
      document.getElementById("gripperValue").innerText = document.getElementById("Gripper").value + "°";
      document.getElementById("elbowValue").innerText = document.getElementById("Elbow").value + "°";
      document.getElementById("shoulderValue").innerText = document.getElementById("Shoulder").value + "°";
      document.getElementById("baseValue").innerText = document.getElementById("Base").value + "°";
    };
  </script>
</body>
</html>
)HTMLHOMEPAGE";

// Hàm xử lý request đến trang chủ
void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", htmlHomePage); // Gửi trang HTML về client
}

// Hàm xử lý khi không tìm thấy trang
void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "File Not Found");
}

// Hàm điều khiển motor
void controlMotor(int in1, int in2, int speed) {
  digitalWrite(in1, speed > 0 ? HIGH : LOW);  // Điều khiển chiều quay
  digitalWrite(in2, speed < 0 ? HIGH : LOW);
}

// Hàm dừng tất cả motor
void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

// Hàm xử lý sự kiện WebSocket
void onRobotArmInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                                  AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:     // Khi client kết nối
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:  // Khi client ngắt kết nối
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {      // Khi nhận dữ liệu từ client
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        std::string myData = "";
        myData.assign((char *)data, len);
        std::istringstream ss(myData);
        std::string key, value;
        std::getline(ss, key, ',');  // Tách key từ dữ liệu
        std::getline(ss, value, ','); // Tách value từ dữ liệu
        
        // Xử lý lệnh điều khiển
        if (key == "Forward") controlState.forward = (value == "1");
        else if (key == "Backward") controlState.backward = (value == "1");
        else if (key == "Left") controlState.left = (value == "1");
        else if (key == "Right") controlState.right = (value == "1");
        else if (key == "Base") writeServoValues(0, atoi(value.c_str()));
        else if (key == "Shoulder") writeServoValues(1, atoi(value.c_str()));
        else if (key == "Elbow") writeServoValues(2, atoi(value.c_str()));
        else if (key == "Gripper") writeServoValues(3, atoi(value.c_str()));
        else if (key == "RotateServos") {  // Xử lý lệnh quay Servo tự động
          if (!servoControl.isRotating) {
            servoControl.shouldRotate = true;
            servoControl.rotationState = !servoControl.rotationState;
            if (servoControl.rotationState) {
              servoControl.targetAngle5 = 100;
              servoControl.targetAngle6 = 180;
            } else {
              servoControl.targetAngle5 = 0;
              servoControl.targetAngle6 = 0;
            }
            servoControl.servo6StartTime = millis();
          }
        }
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;
  }
}

// Hàm ghi giá trị góc cho Servo
void writeServoValues(int servoIndex, int value) {
  servoPins[servoIndex].servo.write(value); // Đặt góc cho Servo
}

// Hàm thiết lập chế độ cho các chân
void setUpPinModes() {
  // Khởi tạo các Servo
  for (int i = 0; i < servoPins.size(); i++) {
    servoPins[i].servo.attach(servoPins[i].servoPin); // Gắn Servo vào chân
    servoPins[i].servo.write(servoPins[i].initialPosition); // Đặt vị trí ban đầu
  }

  // Thiết lập chế độ cho các chân motor
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
}

// Hàm xử lý quay Servo tự động
void handleServoRotation() {
  if (!servoControl.shouldRotate) return; // Nếu không cần quay thì thoát

  unsigned long currentMillis = millis();
  
  // Kiểm tra đã đến thời điểm cập nhật chưa
  if (currentMillis - servoControl.previousMillis >= servoControl.interval) {
    servoControl.previousMillis = currentMillis;
    
    if (!servoControl.isRotating) {
      servoControl.isRotating = true; // Đánh dấu đang quay
    }
    
    // Xử lý quay Servo 5
    if (servoControl.currentAngle5 < servoControl.targetAngle5) {
      servoControl.currentAngle5 = min(servoControl.currentAngle5 + servoControl.stepSize, servoControl.targetAngle5);
      writeServoValues(4, servoControl.currentAngle5);
    } else if (servoControl.currentAngle5 > servoControl.targetAngle5) {
      servoControl.currentAngle5 = max(servoControl.currentAngle5 - servoControl.stepSize, servoControl.targetAngle5);
      writeServoValues(4, servoControl.currentAngle5);
    }
    
    // Xử lý quay Servo 6 với độ trễ
    if (servoControl.rotationState) {
      if (currentMillis - servoControl.servo6StartTime >= servoControl.servo6Delay) {
        if (servoControl.currentAngle6 < servoControl.targetAngle6) {
          servoControl.currentAngle6 = min(servoControl.currentAngle6 + servoControl.stepSize, servoControl.targetAngle6);
          writeServoValues(5, servoControl.currentAngle6);
        }
      }
    } else {
      if (servoControl.currentAngle6 > servoControl.targetAngle6) {
        servoControl.currentAngle6 = max(servoControl.currentAngle6 - servoControl.stepSize, servoControl.targetAngle6);
        writeServoValues(5, servoControl.currentAngle6);
      }
    }
    
    // Kiểm tra đã quay xong chưa
    if (servoControl.currentAngle5 == servoControl.targetAngle5 && 
        servoControl.currentAngle6 == servoControl.targetAngle6) {
      servoControl.shouldRotate = false;
      servoControl.isRotating = false;
    }
  }
}

// Hàm điều khiển xe dựa trên trạng thái
void controlVehicle() {
  if (controlState.forward && !controlState.backward && !controlState.left && !controlState.right) {
    // Tiến
    controlMotor(MOTOR_A_IN1, MOTOR_A_IN2, 234);
    controlMotor(MOTOR_B_IN1, MOTOR_B_IN2, -234);
  } 
  else if (controlState.backward && !controlState.forward && !controlState.left && !controlState.right) {
    // Lùi
    controlMotor(MOTOR_A_IN1, MOTOR_A_IN2, -234);
    controlMotor(MOTOR_B_IN1, MOTOR_B_IN2, 234);
  } 
  else if (controlState.left && !controlState.forward && !controlState.backward && !controlState.right) {
    // Rẽ trái
    controlMotor(MOTOR_A_IN1, MOTOR_A_IN2, -234);
    controlMotor(MOTOR_B_IN1, MOTOR_B_IN2, -234);
  } 
  else if (controlState.right && !controlState.forward && !controlState.backward && !controlState.left) {
    // Rẽ phải
    controlMotor(MOTOR_A_IN1, MOTOR_A_IN2, 234);
    controlMotor(MOTOR_B_IN1, MOTOR_B_IN2, 234);
  } 
  else {
    // Dừng
    stopMotors();
  }
}

// Hàm setup chạy một lần khi khởi động
void setup() {
  setUpPinModes();       // Thiết lập chế độ các chân
  Serial.begin(115200);  // Khởi tạo Serial

  // Cấu hình WiFi
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.softAP(ssid, password); // Khởi tạo Access Point
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Thiết lập các route cho web server
  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  // Thiết lập sự kiện WebSocket
  wsRobotArmInput.onEvent(onRobotArmInputWebSocketEvent);
  server.addHandler(&wsRobotArmInput);

  server.begin(); // Khởi động web server
  Serial.println("HTTP server started");
}

// Hàm loop chạy liên tục
void loop() {
  wsRobotArmInput.cleanupClients(); // Dọn dẹp các client không hoạt động
  controlVehicle();                // Điều khiển xe
  handleServoRotation();           // Xử lý quay Servo tự động
}
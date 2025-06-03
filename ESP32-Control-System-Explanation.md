# 🚗 Giải Thích Hệ Thống Điều Khiển ESP32 Robocar

## 🎯 Tổng Quan Hệ Thống

Hệ thống điều khiển ESP32 Robocar sử dụng kiến trúc phân tầng với các thành phần chính:

### 1. **Tầng Phần Cứng (Hardware Layer)**
- **ESP32 DevKit**: Vi điều khiển chính với WiFi tích hợp
- **L298N Motor Driver**: Điều khiển động cơ DC với H-Bridge
- **Servo Motors**: Điều khiển cánh tay robot (6 servo)
- **Line Sensors**: Cảm biến đường cho xe tự lái
- **Power System**: Nguồn 18265x3 (11.1 V) cho toàn bộ hệ thống

### 2. **Tầng Giao Tiếp (Communication Layer)**
- **WiFi Access Point**: ESP32 tạo mạng WiFi riêng
- **WebSocket Protocol**: Truyền dữ liệu real-time
- **HTTP Server**: Phục vụ giao diện web điều khiển

---

## 🔧 Chi Tiết Hoạt Động Từng Thành Phần

### 🎮 **Hệ Thống RC Car**

#### **WiFi Access Point Mode**
```cpp
// ESP32 tạo mạng WiFi riêng
WiFi.softAP(ap_ssid, ap_password);
IPAddress ap_local_IP(192, 168, 4, 1);
```

**Cách hoạt động:**
1. ESP32 khởi tạo chế độ Access Point
2. Tạo mạng WiFi có tên "Mat ngu trien mien"
3. Thiết lập địa chỉ IP tĩnh 192.168.4.1
4. Cho phép các thiết bị kết nối đến mạng này

#### **WebSocket Communication**
```cpp
WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_TEXT:
            // Xử lý lệnh nhận được
            String message = String((char*)payload);
            processCommand(message);
            break;
    }
}
```

**Quy trình hoạt động:**
1. **Thiết lập kết nối**: Client kết nối đến WebSocket port 81
2. **Nhận lệnh**: Server nhận lệnh từ web interface
3. **Phân tích lệnh**: Parse chuỗi lệnh (ví dụ: "Forward,1")
4. **Thực thi**: Gọi hàm điều khiển motor/servo tương ứng
5. **Phản hồi**: Gửi trạng thái về client (optional)

#### **Motor Control System**
```cpp
void CAR_moveForward() {
    // Thiết lập hướng quay
    digitalWrite(IN1_PIN, HIGH);  // Right motor forward
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);  // Left motor forward
    digitalWrite(IN4_PIN, LOW);
    
    // Thiết lập tốc độ PWM
    analogWrite(ENA_PIN, MOTOR_BASE_SPEED);  // Right motor speed
    analogWrite(ENB_PIN, MOTOR_BASE_SPEED);  // Left motor speed
}
```

**Nguyên lý H-Bridge L298N:**
- **IN1/IN2**: Điều khiển hướng quay motor phải
- **IN3/IN4**: Điều khiển hướng quay motor trái
- **ENA/ENB**: Điều khiển tốc độ bằng PWM (0-255)

**Logic điều khiển:**
- **Tiến**: IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW
- **Lùi**: IN1=LOW, IN2=HIGH, IN3=LOW, IN4=HIGH
- **Rẽ trái**: IN1=HIGH, IN2=LOW, IN3=LOW, IN4=HIGH
- **Rẽ phải**: IN1=LOW, IN2=HIGH, IN3=HIGH, IN4=LOW

**Tính năng servo:**
- **3-4 servo motors**: Shoulder, Elbow, Gripper
- **Smooth movement**: Di chuyển từ từ tránh giật
- **Position control**: Điều khiển góc 0-180°

### 🤖 **Hệ Thống Self-Driving Car**

#### **Line Following Algorithm**
```cpp
int readSensors() {
    int leftMost = digitalRead(LEFT_MOST);
    int leftInner = digitalRead(LEFT_INNER);
    int rightInner = digitalRead(RIGHT_INNER);
    int rightMost = digitalRead(RIGHT_MOST);
    
    // Phân tích pattern sensor
    if (leftMost == 1) return -2;  // Hard left turn
    if (leftInner == 1) return -1; // Soft left turn
    if (rightInner == 1) return 1; // Soft right turn
    if (rightMost == 1) return 2;  // Hard right turn
    
    return 0; // Go straight
}
```

**Thuật toán đi theo đường:**
1. **Đọc sensors**: 4 sensor đọc đường màu đen/trắng
2. **Pattern recognition**: Nhận dạng vị trí xe so với đường
3. **Decision making**: Quyết định hướng rẽ dựa trên pattern
4. **Motor control**: Điều khiển motor theo quyết định
5. **Continuous loop**: Lặp lại quá trình với tần số cao

#### **Intelligent Turning System**
```cpp
void setMotors(int leftSpeed, int rightSpeed) {
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Set direction pins
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    
    // Apply PWM speed
    ledcWrite(ENA, leftSpeed);
    digitalWrite(ENB, rightSpeed > 0 ? HIGH : LOW);
}
```

**Các loại rẽ thông minh:**
- **Soft Turn**: Rẽ nhẹ, giảm tốc độ một bánh
- **Hard Turn**: Rẽ mạnh, đảo chiều một bánh
- **Recovery Turn**: Tìm lại đường khi mất vết

---

**Web server workflow:**
1. Client request đến "/"
2. Server gửi HTML content
3. HTML load và kết nối WebSocket
4. Real-time control qua WebSocket

---

## ⚙️ **Cấu Hình Hệ Thống**

### **PWM Configuration**
```cpp
// PWM properties for ESP32
const int freq = 5000;        // 5kHz frequency
const int resolution = 8;     // 8-bit resolution (0-255)

// Setup PWM channels
ledcAttach(ENA, freq, resolution);
ledcAttach(ENB, freq, resolution);
```

### **Timing & Performance**
- **Loop frequency**: ~40Hz (25ms delay)
- **Servo update rate**: 200Hz (5ms interval)
- **WebSocket latency**: <50ms
- **Sensor reading**: Real-time (no delay)

### **Memory Management**
- **HTML storage**: Stored in PROGMEM
- **WebSocket buffers**: Dynamic allocation
- **Servo positions**: Static variables
- **Debug output**: Serial buffer management

---

## 🔍 **Troubleshooting Guide**

### **Common Issues & Solutions**

#### **WiFi Connection Problems**
```bash
# Kiểm tra:
1. ESP32 power LED sáng
2. Serial Monitor hiển thị "AP IP address: 192.168.4.1"
3. Device kết nối đúng mạng WiFi
4. Browser cache cleared

# Giải pháp:
- Reset ESP32 và chờ 30 giây
- Thử browser khác
- Kiểm tra firewall settings
```

#### **Servo Problems**
```bash
# Power issues:
- Adequate power supply (6V/2A minimum)
- External regulator for servo power
- Voltage drop under load

# Signal issues:
- Proper servo signal wire connections
- EMI from motors
- Grounding between ESP32 and servos
```

### **Debug Commands**
```cpp
// Enable debug mode
#define DEBUG_MODE true

// Serial output examples
Serial.print("Sensor values: ");
Serial.print("Motor speeds: L=");
Serial.print("Servo position: ");
```

---
**🎯 Kết Luận**

Hệ thống điều khiển ESP32 Robocar là một giải pháp tích hợp hoàn chỉnh, kết hợp:
- **Hardware control** thông qua PWM và GPIO
- **Network communication** qua WiFi và WebSocket
- **Real-time processing** với thuật toán thông minh


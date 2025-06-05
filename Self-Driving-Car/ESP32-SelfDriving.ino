/* ESP32
 * Xe Tự Lái - Thuật Toán Đi Theo Đường Kẻ Đơn Giản
 * Sử dụng driver motor L298N và cảm biến đường kẻ số
 */

#include <Arduino.h>

// Định nghĩa chân cho driver motor L298N
#define ENA 5  // Điều khiển PWM cho motor phải
#define ENB 18  // Điều khiển PWM cho motor trái
#define IN1 2   // Hướng motor phải 1
#define IN2 4   // Hướng motor phải 2
#define IN3 16  // Hướng motor trái 1
#define IN4 17  // Hướng motor trái 2

// Định nghĩa chân cho cảm biến đường kẻ DIGITAL
#define LEFT_MOST 27  // Cảm biến ngoài cùng bên trái
#define LEFT_INNER 14 // Cảm biến trong bên trái
#define RIGHT_INNER 12 // Cảm biến trong bên phải
#define RIGHT_MOST 13 // Cảm biến ngoài cùng bên phải

// Biến tốc độ motor
int baseSpeed = 255;  // Tốc độ cơ bản cho motor, phạm vi 0-255
int turnSpeed = 212;   // Tốc độ thường cho bánh trong khi rẽ nhẹ
int hardTurnSpeed = 190; // Tốc độ ngược cho bánh trong khi rẽ mạnh
#define DEBUG_MODE true

// Thuộc tính PWM cho ESP32
const int freq = 5000; // Tần số PWM 5kHz
const int resolution = 8;  // Độ phân giải 8-bit, 0-255

// Cho khôi phục khẩn cấp
bool finishLineDetected = false; // ĐỪNG THAY ĐỔI
// Biến này được sử dụng để chỉ báo khi phát hiện vạch đích

// Cho các mẫu nhấp nháy LED
unsigned long lastLedToggle = 0;
int ledBlinkInterval = 500; // Khoảng thời gian nhấp nháy mặc định tính bằng ms

// Cho khôi phục khẩn cấp
#define RECOVERY_TURN_DIRECTION 1  // 1 = rẽ phải, -1 = rẽ trái

// Cho logic rẽ cải tiến
int lastActiveSensor = 0; // 0=không có, -2=trái nhất, -1=trái trong, 1=phải trong, 2=phải nhất
bool inHardLeftTurn = false;
bool inHardRightTurn = false;
unsigned long turnStartTime = 0;
const unsigned long MIN_TURN_TIME = 0; // Thời gian rẽ tối thiểu tính bằng milliseconds

void setup() {
    // Khởi tạo serial để debug
    Serial.begin(115200);
    Serial.println("ESP32 Self-Driving Car starting...");
    
    // Bỏ cấu hình LED (không cần thiết)
    // pinMode(LED_PIN, OUTPUT);
    // digitalWrite(LED_PIN, HIGH); // Tắt LED (có thể bị đảo trên một số board)
    
    // Cấu hình các chân điều khiển motor
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Cấu hình PWM của ESP32 để điều khiển motor sử dụng hàm ledcAttach mới
    // Tự động chọn channel phù hợp
    ledcAttach(ENA, freq, resolution);  // Tự động gán channel cho ENA
    ledcAttach(ENB, freq, resolution);  // Tự động gán channel cho ENB
    
    // Khởi tạo các chân cảm biến đường kẻ
    pinMode(LEFT_MOST, INPUT);
    pinMode(LEFT_INNER, INPUT);
    pinMode(RIGHT_INNER, INPUT);
    pinMode(RIGHT_MOST, INPUT);
    
    // Đếm ngược trước khi bắt đầu - không có LED nhấp nháy
    Serial.println("Starting in:");
    for (int i = 3; i > 0; i--) {
        Serial.println(i);
        delay(1000);
    }
    
    Serial.println("GO!");
}

// Hàm điều khiển motor
void setMotors(int leftSpeed, int rightSpeed) {
    // Giới hạn tốc độ trong phạm vi PWM hợp lệ
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Motor trái - điều khiển bằng IN3, IN4
    if (leftSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        leftSpeed = -leftSpeed;  // Chuyển tốc độ âm thành giá trị PWM dương
    }
    
    // Motor phải - điều khiển bằng IN1, IN2
    if (rightSpeed >= 0) {
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        rightSpeed = -rightSpeed;  // Chuyển tốc độ âm thành giá trị PWM dương
    }
    
    // Áp dụng điều khiển tốc độ PWM sử dụng ledcWrite dựa trên pin cho API mới
    ledcWrite(ENB, leftSpeed);   // Ghi vào pin ENB cho motor trái
    ledcWrite(ENA, rightSpeed);  // Ghi vào pin ENA cho motor phải
    
    if (DEBUG_MODE) {
        Serial.print("Motors: L=");
        Serial.print(leftSpeed);
        Serial.print(" R=");
        Serial.println(rightSpeed);
    }
}

// Hàm dừng motor
void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENA, 0); // Dừng motor phải
    ledcWrite(ENB, 0); // Dừng motor trái
    Serial.println("Motors stopped");
}

// Đọc giá trị cảm biến đường kẻ và trả về quyết định điều khiển
int readSensors() {
    // Đọc giá trị số từ 4 cảm biến
    // ĐÃ SỬA: Đọc số: 1 cho đường kẻ đen, 0 cho bề mặt trắng
    int leftMost = digitalRead(LEFT_MOST);
    int leftInner = digitalRead(LEFT_INNER);
    int rightInner = digitalRead(RIGHT_INNER);
    int rightMost = digitalRead(RIGHT_MOST);
    
    if (DEBUG_MODE) {
        Serial.print("Sensors: LM=");
        Serial.print(leftMost);
        Serial.print(" LI=");
        Serial.print(leftInner);
        Serial.print(" RI=");
        Serial.print(rightInner);
        Serial.print(" RM=");
        Serial.println(rightMost);
    }

    // Kiểm tra vạch đích (tất cả cảm biến đọc đen)
    if (leftMost == 1 && leftInner == 1 && rightInner == 1 && rightMost == 1) {
        // Thêm debounce để tránh phát hiện sai
        delay(50);
        
        // Đọc lại để xác nhận
        leftMost = digitalRead(LEFT_MOST);
        leftInner = digitalRead(LEFT_INNER);
        rightInner = digitalRead(RIGHT_INNER);
        rightMost = digitalRead(RIGHT_MOST);
        
        // Chỉ dừng nếu vẫn phát hiện vạch đích
        if (leftMost == 1 && leftInner == 1 && rightInner == 1 && rightMost == 1) {
            finishLineDetected = true;
            return 0; // Dừng
        }
    }

    // Kiểm tra "ra khỏi đường" - tất cả cảm biến đọc trắng
    if (leftMost == 0 && leftInner == 0 && rightInner == 0 && rightMost == 0) {
        if (DEBUG_MODE) {
            Serial.println("All sensors white - continuing last direction");
        }
        
        // Tiếp tục rẽ theo hướng cuối cùng cho đến khi tìm thấy đường lại
        if (lastActiveSensor < 0) {
            return -2; // Tiếp tục rẽ trái dựa trên phát hiện cuối
        } else if (lastActiveSensor > 0) {
            return 2; // Tiếp tục rẽ phải dựa trên phát hiện cuối
        } else {
            return 10; // Đi thẳng nếu không có hướng trước
        }
    }

    // Xử lý chuyển tiếp rẽ và theo dõi cảm biến
    
    // Rẽ trái mạnh được khởi tạo bởi cảm biến trái nhất
    if (leftMost == 1) {
        lastActiveSensor = -2;
        inHardLeftTurn = true;
        inHardRightTurn = false;
        turnStartTime = millis();
        return -2; // Rẽ trái mạnh
    }
    
    // Rẽ phải mạnh được khởi tạo bởi cảm biến phải nhất
    if (rightMost == 1) {
        lastActiveSensor = 2;
        inHardRightTurn = true;
        inHardLeftTurn = false;
        turnStartTime = millis();
        return 2; // Rẽ phải mạnh
    }

    // Kiểm tra nếu nên thoát khỏi rẽ mạnh khi cảm biến trong phát hiện đường
    if (inHardLeftTurn && leftInner == 1) {
        // Thoát khỏi rẽ trái mạnh nếu cảm biến trái trong thấy đường lại
        // và thời gian rẽ tối thiểu đã qua
        if (millis() - turnStartTime > MIN_TURN_TIME) {
            inHardLeftTurn = false;
            lastActiveSensor = -1;
            return -1; // Chuyển sang rẽ trái nhẹ
        } else {
            return -2; // Tiếp tục rẽ trái mạnh cho đến thời gian tối thiểu
        }
    }
    
    if (inHardRightTurn && rightInner == 1) {
        // Thoát khỏi rẽ phải mạnh nếu cảm biến phải trong thấy đường lại
        // và thời gian rẽ tối thiểu đã qua
        if (millis() - turnStartTime > MIN_TURN_TIME) {
            inHardRightTurn = false;
            lastActiveSensor = 1;
            return 1; // Chuyển sang rẽ phải nhẹ
        } else {
            return 2; // Tiếp tục rẽ phải mạnh cho đến thời gian tối thiểu
        }
    }
    
    // Tiếp tục rẽ mạnh nếu đang trong một lần rẽ
    if (inHardLeftTurn) {
        return -2;
    }
    if (inHardRightTurn) {
        return 2;
    }
    
    // Theo đường bình thường cho rẽ nhẹ
    if (leftInner == 1) {
        lastActiveSensor = -1;
        return -1; // Rẽ trái nhẹ - cảm biến trái phát hiện đường
    }
    
    if (rightInner == 1) {
        lastActiveSensor = 1;
        return 1; // Rẽ phải nhẹ - cảm biến phải phát hiện đường
    }
    
    // Mặc định - đi thẳng
    return 10; // Tiến
}

// Kiểm tra lệnh serial
void checkSerialCommands() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        Serial.println("Lệnh không xác định. Các lệnh có sẵn:");
        Serial.println("  Hiện tại không có lệnh nào");
    }
}

void loop() {
    // Kiểm tra lệnh serial
    checkSerialCommands();
    
    // Đọc vị trí đường kẻ
    int controlDecision = readSensors();

    // Kiểm tra phát hiện vạch đích
    if (finishLineDetected) {
        stopMotors();
        
        // Không sử dụng LED để chỉ báo vạch đích
        if (DEBUG_MODE) {
            Serial.println("Finish line detected! Car stopped.");
        }
        return;
    }

    // Áp dụng quyết định điều khiển cho motor
    switch (controlDecision) {
        case 1: // Rẽ phải nhẹ
            setMotors(baseSpeed, turnSpeed); // Motor trái nhanh hơn, motor phải chậm hơn để rẽ phải
            if (DEBUG_MODE) Serial.println("Slight right turn");
            break;
            
        case -1: // Rẽ trái nhẹ
            setMotors(turnSpeed, baseSpeed); // Motor trái chậm hơn, motor phải nhanh hơn để rẽ trái
            if (DEBUG_MODE) Serial.println("Slight left turn");
            break;
            
        case 2: // Rẽ phải mạnh
            setMotors(baseSpeed, -hardTurnSpeed); // Motor trái tiến, motor phải lùi để rẽ mạnh
            if (DEBUG_MODE) Serial.println("Hard right turn");
            break;
            
        case -2: // Rẽ trái mạnh
            setMotors(-hardTurnSpeed, baseSpeed); // Motor trái lùi, motor phải tiến để rẽ mạnh
            if (DEBUG_MODE) Serial.println("Hard left turn");
            break;
            
        case 0: // Dừng (phát hiện vạch đích)
            stopMotors();
            if (finishLineDetected) {
                if (DEBUG_MODE) Serial.println("Finish line detected! Car stopped.");
            }
            break;
            
        case 3: // Rẽ phải khôi phục
            setMotors(baseSpeed, -turnSpeed);
            if (DEBUG_MODE) Serial.println("Recovery right turn");
            break;
            
        case -3: // Rẽ trái khôi phục
            setMotors(-turnSpeed, baseSpeed);
            if (DEBUG_MODE) Serial.println("Recovery left turn");
            break;
            
        default: // Tiến
            setMotors(baseSpeed, baseSpeed);
            if (DEBUG_MODE) Serial.println("Going forward");
            break;
    }

    // Delay ngắn
    delay(20); // Điều chỉnh theo nhu cầu về độ phản hồi
}
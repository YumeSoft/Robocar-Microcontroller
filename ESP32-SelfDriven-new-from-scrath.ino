/* ESP32
 * Xe Tự Lái - Đi Theo Đường Kẻ Đơn Giản
 * Sử dụng driver động cơ L298N và cảm biến đường kẻ số
 */

#include <Arduino.h>

// Định nghĩa chân cho driver động cơ L298N
#define ENA 5    // Điều khiển PWM cho động cơ phải
#define ENB 18   // Điều khiển PWM cho động cơ trái
#define IN1 2    // Hướng động cơ phải 1
#define IN2 4    // Hướng động cơ phải 2
#define IN3 16   // Hướng động cơ trái 1
#define IN4 17   // Hướng động cơ trái 2

// Định nghĩa chân cho cảm biến đường kẻ (cảm biến số)
#define RIGHT_MOST 13   // Cảm biến ngoài cùng bên phải
#define RIGHT_INNER 12  // Cảm biến trong bên phải
#define LEFT_INNER 14   // Cảm biến trong bên trái
#define LEFT_MOST 27    // Cảm biến ngoài cùng bên trái


// Chân LED báo hiệu (T2)
#define LED_PIN T2

// Biến tốc độ động cơ
int baseSpeed = 255;      // Tốc độ cơ bản, phạm vi 0-255
int turnSpeed = 130;      // Tốc độ cho bánh trong khi rẽ nhẹ
int hardTurnSpeed = 200;  // Tốc độ cho bánh trong khi rẽ gắt (lùi)

// Biến trạng thái
bool programStopped = false;
unsigned long lastFlashTime = 0;
bool ledState = false;

// Biến lưu hướng rẽ cuối cùng
int lastDirection = 0; // 0: thẳng, -1: trái, 1: phải

void setup() {
  Serial.begin(115200);
  Serial.println("Khởi động xe tự lái...");
  
  // Cấu hình chân động cơ
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Cấu hình chân cảm biến
  pinMode(LEFT_MOST, INPUT);
  pinMode(LEFT_INNER, INPUT);
  pinMode(RIGHT_INNER, INPUT);
  pinMode(RIGHT_MOST, INPUT);
  
  // Cấu hình chân LED
  pinMode(LED_PIN, OUTPUT);
  
  // Dừng động cơ ban đầu
  stopMotors();
  
  delay(2000); // Chờ 2 giây trước khi bắt đầu
  Serial.println("Bắt đầu đi theo đường kẻ!");
}

void loop() {
  if (programStopped) {
    // Nháy LED khi chương trình dừng
    flashLED();
    return;
  }
  
  // Đọc trạng thái cảm biến (1 = phát hiện đường đen, 0 = không phát hiện)
  bool leftMost = digitalRead(LEFT_MOST) == HIGH;
  bool leftInner = digitalRead(LEFT_INNER) == HIGH;
  bool rightInner = digitalRead(RIGHT_INNER) == HIGH;
  bool rightMost = digitalRead(RIGHT_MOST) == HIGH;
  
  // In trạng thái cảm biến để debug
  Serial.print("Cảm biến: ");
  Serial.print(leftMost ? "1" : "0");
  Serial.print(leftInner ? "1" : "0");
  Serial.print(rightInner ? "1" : "0");
  Serial.println(rightMost ? "1" : "0");
  
  // Kiểm tra vạch đích (tất cả cảm biến phát hiện đường đen)
  if (leftMost && leftInner && rightInner && rightMost) {
    Serial.println("Phát hiện vạch đích! Dừng xe...");
    stopMotors();
    programStopped = true;
    return;
  }
  
  // Logic đi theo đường kẻ
  if (!leftMost && !leftInner && !rightInner && !rightMost) {
    // Không cảm biến nào phát hiện đường -> đi theo hướng cuối cùng
    if (lastDirection == -1) {
      Serial.println("Mất đường - rẽ trái theo hướng cuối");
      moveForward(turnSpeed, baseSpeed);
    } else if (lastDirection == 1) {
      Serial.println("Mất đường - rẽ phải theo hướng cuối");
      moveForward(baseSpeed, turnSpeed);
    } else {
      Serial.println("Đi thẳng");
      moveForward(baseSpeed, baseSpeed);
    }
  }
  else if (leftInner && rightInner && !leftMost && !rightMost) {
    // Chỉ hai cảm biến trong phát hiện đường -> đi thẳng nhanh
    Serial.println("Đi thẳng (giữa đường)");
    moveForward(baseSpeed, baseSpeed);
  }
  else if (leftInner && !rightInner && !leftMost && !rightMost) {
    // Chỉ cảm biến trái trong phát hiện -> re trai nhe va set lastDirection = -1
    Serial.println("Rẽ trai gắt");
    moveForward(baseSpeed, turnSpeed);
    lastDirection = -1;
  }
  else if (!leftInner && rightInner && !leftMost && !rightMost) {
    // Chỉ cảm biến phải trong phát hiện -> rẽ phai nhe va set lastDirection = 1
    Serial.println("Rẽ phai nhe");
    moveForward(turnSpeed, baseSpeed);
    lastDirection = 1;
  }
  else if (leftMost && !leftInner && !rightInner && !rightMost) {
    // Chỉ cảm biến ngoài cùng trái phát hiện -> rẽ trái cực gắt
    Serial.println("Rẽ trái CỰC GẮT");
    hardTurnLeft();
    lastDirection = -1;
  }
  else if (!leftMost && !leftInner && !rightInner && rightMost) {
    // Chỉ cảm biến ngoài cùng phải phát hiện -> rẽ phải cực gắt
    Serial.println("Rẽ phải CỰC GẮT");
    hardTurnRight();
    lastDirection = 1;
  }
  else if (leftMost && leftInner && !rightInner && !rightMost) {
    // Hai cảm biến bên trái phát hiện -> rẽ trái mạnh
    Serial.println("Rẽ trai mạnh");
    hardTurnLeft();
    lastDirection = -1;
  }
  else if (!leftMost && !leftInner && rightInner && rightMost) {
    // Hai cảm biến bên phải phát hiện -> rẽ phải mạnh
    Serial.println("Rẽ phai mạnh");
    hardTurnRight();
    lastDirection = 1;
  }
  else if (leftInner && rightInner) {
    // Cả hai cảm biến trong phát hiện (có thể có thêm ngoài) -> đi thẳng
    Serial.println("Đi thẳng (có cảm biến ngoài)");
    moveForward(baseSpeed, baseSpeed);
  }
  else {
    // Trường hợp khác -> dừng lại một chút rồi tìm đường
    Serial.println("Trường hợp khác - dừng và tìm đường");
    stopMotors();
    delay(100);
    moveForward(baseSpeed, baseSpeed);
  }
  
  delay(50); // Độ trễ nhỏ để ổn định
}

// Hàm di chuyển tiến
void moveForward(int leftSpeed, int rightSpeed) {
  // Động cơ trái tiến
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, leftSpeed);
  
  // Động cơ phải tiến
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, rightSpeed);
}

// Hàm rẽ trái gắt (động cơ trái lùi, phải tiến)
void hardTurnLeft() {
  // Động cơ trái lùi
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, hardTurnSpeed);
  
  // Động cơ phải tiến
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, baseSpeed);
}

// Hàm rẽ phải gắt (động cơ phải lùi, trái tiến)
void hardTurnRight() {
  // Động cơ trái tiến
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, baseSpeed);
  
  // Động cơ phải lùi
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, hardTurnSpeed);
}

// Hàm dừng động cơ
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Hàm nháy LED báo hiệu
void flashLED() {
  unsigned long currentTime = millis();
  if (currentTime - lastFlashTime >= 500) { // Nháy mỗi 500ms
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastFlashTime = currentTime;
    Serial.println(ledState ? "LED BẬT" : "LED TẮT");
  }
}
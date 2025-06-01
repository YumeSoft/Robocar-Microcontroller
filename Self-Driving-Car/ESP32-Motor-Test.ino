/* ESP32
 * Self Driving Car - Line Following with PID Control
 * Using L298N motor driver, HW-871 line sensors, and HC-SR04 sonar sensor
 */

#include <Arduino.h>

// Pin definitions for L298N motor driver
#define ENA 19  // PWM control for left motors
#define ENB 21  // PWM control for right motors
#define IN1 2   // Left motors direction 1
#define IN2 4   // Left motors direction 2
#define IN3 16  // Right motors direction 1
#define IN4 17  // Right motors direction 2

#define LED_PIN T2

// Pin definitions for HW-871 line sensors
#define SENSOR_1 26  // Leftmost sensor
#define SENSOR_2 25
#define SENSOR_3 33  // Middle sensor
#define SENSOR_4 32
#define SENSOR_5 35  // Rightmost sensor
// Motor speed variables
int baseSpeed = 150;  // Base speed for motors, range 0-255

// Test sequence status
int testSequence = 0;
unsigned long testTimer = 0;
bool testRunning = true;

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    Serial.println("Motor Test Starting...");
    
    // Initialize motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Initialize PWM pins with correct frequency
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Make sure PWM is off initially
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    
    // Set all direction pins to LOW initially
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED is active LOW on STM32 Blue Pill
    
    // Quick test of LED to show program is running
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    
    // Small delay before starting motor tests
    delay(1000);
    
    Serial.println("Setup complete, beginning motor tests...");
    testTimer = millis();
}

void loop() {
    // Change test sequence every 2 seconds
    if (millis() - testTimer >= 2000) {
        testTimer = millis();
        testSequence++;
        if (testSequence > 5) {
            testSequence = 1;  // Reset to first test
        }
        
        // Blink LED to indicate new test sequence
        digitalWrite(LED_PIN, LOW);  // LED on
        delay(100);
        digitalWrite(LED_PIN, HIGH); // LED off
    }
    
    // Execute current test sequence
    switch (testSequence) {
        case 1:  // Forward
            moveForward();
            break;
        case 2:  // Backward
            moveBackward();
            break;
        case 3:  // Turn left
            turnLeft();
            break;
        case 4:  // Turn right
            turnRight();
            break;
        case 5:  // Stop
            stopMotors();
            break;
    }
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, baseSpeed);
    analogWrite(ENB, baseSpeed);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, baseSpeed);
    analogWrite(ENB, baseSpeed);
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, baseSpeed);
    analogWrite(ENB, baseSpeed);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, baseSpeed);
    analogWrite(ENB, baseSpeed);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
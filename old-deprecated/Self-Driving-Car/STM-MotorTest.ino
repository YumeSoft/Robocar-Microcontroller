/* STM32 Blue Pill
 * Motor Controller Test Program
 * Using L298N motor driver
 */

#include <Arduino.h>

// Pin definitions for L298N motor driver (using A1, A2, A3, A4)

#define ENA PA1  // PWM control for left motors
#define ENB PA2  // PWM control for right motors
#define IN1 PA3  // Left motors direction 1
#define IN2 PA4  // Left motors direction 2
#define IN3 PA5  // Right motors direction 1
#define IN4 PA6  // Right motors direction 2

// Define onboard LED pin
#define LED_PIN PA13  // Onboard LED on STM32 Blue Pill

// PWM frequency and resolution settings
const int PWM_FREQ = 5000; // 5 kHz PWM frequency
// Removed conflicting PWM_RESOLUTION variable since it's already defined in pins_arduino.h
// The STM32 Arduino core already defines PWM_RESOLUTION as 8 bits

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
/*
 * PWM Test Program for L298N Motor Driver
 * This test focuses solely on the PWM functionality for ENA and ENB pins
 */

#include <Arduino.h>

// Pin definitions for L298N motor driver
#define ENA PA1  // PWM control for left motors
#define ENB PA2  // PWM control for right motors
#define IN1 PA3  // Left motors direction 1
#define IN2 PA4  // Left motors direction 2
#define IN3 PA5  // Right motors direction 1
#define IN4 PA6  // Right motors direction 2

// Define PWM properties
const int PWM_FREQ = 5000;       // 5 kHz PWM frequency
const int PWM_RESOLUTION = 8;    // 8-bit resolution (0-255)

// Test variables
int pwmValue = 0;
int pwmStep = 5;
unsigned long lastChangeTime = 0;
const int changeInterval = 100;  // Change PWM every 100ms
bool rampingUp = true;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("PWM Test Starting...");
  
  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize PWM pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Set direction pins to enable motor rotation (forward direction)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Start with PWM at 0
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  Serial.println("Motor direction pins set for forward movement.");
  Serial.println("Will now ramp PWM from 0 to 255 and back.");
  Serial.println("Check if motors start running at some point.");
  
  delay(2000); // Wait before starting the test
}

void loop() {
  // Update PWM value at regular intervals
  if (millis() - lastChangeTime > changeInterval) {
    lastChangeTime = millis();
    
    // Adjust PWM value
    if (rampingUp) {
      pwmValue += pwmStep;
      if (pwmValue >= 255) {
        pwmValue = 255;
        rampingUp = false;
      }
    } else {
      pwmValue -= pwmStep;
      if (pwmValue <= 0) {
        pwmValue = 0;
        rampingUp = true;
        
        // Pause for a moment at zero
        Serial.println("Pausing for 2 seconds before next cycle");
        delay(2000);
      }
    }
    
    // Apply PWM to both motors
    analogWrite(ENA, pwmValue);
    analogWrite(ENB, pwmValue);
    
    // Print current PWM value
    Serial.print("PWM Value: ");
    Serial.println(pwmValue);
  }
}
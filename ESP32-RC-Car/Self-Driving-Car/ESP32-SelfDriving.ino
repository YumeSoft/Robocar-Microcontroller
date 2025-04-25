/* ESP32
 * Self Driving Car - Simple Line Following
 * Using L298N motor driver and digital line sensors
 */

#include <Arduino.h>

// Pin definitions for L298N motor driver
#define ENA 5  // PWM control for right motors
#define ENB 18  // PWM control for left motors
#define IN1 2   // Right motors direction 1
#define IN2 4   // Right motors direction 2
#define IN3 16  // Left motors direction 1
#define IN4 17  // Left motors direction 2

// Pin definitions for line sensors (digital sensors)
#define LEFT_MOST 13  // Leftmost sensor
#define LEFT_INNER 12 // Left inner sensor
#define RIGHT_INNER 14 // Right inner sensor
#define RIGHT_MOST 27 // Rightmost sensor

// Motor speed variables
int baseSpeed = 255;  // Base speed for motors, range 0-255
int turnSpeed = 170;   // Regular speed for inner wheel in slight turns (changed from negative value)
int hardTurnSpeed = -240; // Reversed speed for inner wheel in hard turns
int reducedTurnSpeed = 160; // Speed for the reduced speed side during turns

// Remove wheel locking control variables
#define DEBUG_MODE true

// PWM properties for ESP32
const int freq = 5000;
const int resolution = 8;  // 8-bit resolution, 0-255

// For emergency recovery
bool finishLineDetected = false; // DONT CHANGE THIS
// This variable is used to indicate if the finish line has been detected

// For LED blinking patterns
unsigned long lastLedToggle = 0;
int ledBlinkInterval = 500; // Default blink interval in ms

// For emergency recovery
#define RECOVERY_TURN_DIRECTION 1  // 1 = right turn, -1 = left turn

// For improved turning logic
int lastActiveSensor = 0; // 0=none, -2=leftmost, -1=left inner, 1=right inner, 2=rightmost
bool inHardLeftTurn = false;
bool inHardRightTurn = false;
unsigned long turnStartTime = 0;
const unsigned long MIN_TURN_TIME = 0; // Minimum turn time in milliseconds

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    Serial.println("ESP32 Self-Driving Car starting...");
    
    // Comment out LED configuration
    // pinMode(LED_PIN, OUTPUT);
    // digitalWrite(LED_PIN, HIGH); // Turn off LED (may be inverted on some boards)
    
    // Configure motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Configure ESP32 PWM for motor control using newer ledcAttach function
    // This automatically selects appropriate channels
    ledcAttach(ENA, freq, resolution);  // Automatically assigns a channel for ENA
    ledcAttach(ENB, freq, resolution);  // Automatically assigns a channel for ENB
    
    // Initialize line sensor pins
    pinMode(LEFT_MOST, INPUT);
    pinMode(LEFT_INNER, INPUT);
    pinMode(RIGHT_INNER, INPUT);
    pinMode(RIGHT_MOST, INPUT);
    
    // Countdown before starting - without LED flashing
    Serial.println("Starting in:");
    for (int i = 3; i > 0; i--) {
        Serial.println(i);
        delay(1000);
    }
    
    Serial.println("GO!");
}

// Function to control motors
void setMotors(int leftSpeed, int rightSpeed) {
    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Left motors
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftSpeed = -leftSpeed;  // Convert negative speed to positive PWM value
    }
    
    // Right motors
    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH); 
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        rightSpeed = -rightSpeed;  // Convert negative speed to positive PWM value
    }
    
    // Apply PWM speed control using pin-based ledcWrite for the newer API
    ledcWrite(ENA, leftSpeed);   // Writing to ENA pin for left motors
    ledcWrite(ENB, rightSpeed);  // Writing to ENB pin for right motors
    
    if (DEBUG_MODE) {
        Serial.print("Motors: L=");
        Serial.print(leftSpeed);
        Serial.print(" R=");
        Serial.println(rightSpeed);
    }
}

// Function to stop motors
void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENA, 0); // Stop right motors50
    ledcWrite(ENB, 0); // Stop left motors
    Serial.println("Motors stopped");
}

// Read line sensor values and return control decision
int readSensors() {
    // Read digital values from the 4 sensors
    // CORRECTED: Digital reading: 1 for black line, 0 for white surface
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

    // Check for finish line (all sensors reading black)
    if (leftMost == 1 && leftInner == 1 && rightInner == 1 && rightMost == 1) {
        // Added debounce to prevent false detections
        delay(50);
        
        // Read again to confirm
        leftMost = digitalRead(LEFT_MOST);
        leftInner = digitalRead(LEFT_INNER);
        rightInner = digitalRead(RIGHT_INNER);
        rightMost = digitalRead(RIGHT_MOST);
        
        // Only stop if still detecting finish line
        if (leftMost == 1 && leftInner == 1 && rightInner == 1 && rightMost == 1) {
            finishLineDetected = true;
            return 0; // Stop
        }
    }

    // Check for "out of track" - all sensors reading white
    if (leftMost == 0 && leftInner == 0 && rightInner == 0 && rightMost == 0) {
        if (DEBUG_MODE) {
            Serial.println("All sensors white - continuing last direction");
        }
        
        // Continue turning in the last direction until line is found again
        if (lastActiveSensor < 0) {
            return -2; // Continue turning left based on last detection
        } else if (lastActiveSensor > 0) {
            return 2; // Continue turning right based on last detection
        } else {
            return 10; // Forward if no previous direction
        }
    }

    // Handle turn transitions and sensor tracking
    
    // Hard left turn initiated by leftmost sensor
    if (leftMost == 1) {
        lastActiveSensor = -2;
        inHardLeftTurn = true;
        inHardRightTurn = false;
        turnStartTime = millis();
        return -2; // Hard left turn
    }
    
    // Hard right turn initiated by rightmost sensor
    if (rightMost == 1) {
        lastActiveSensor = 2;
        inHardRightTurn = true;
        inHardLeftTurn = false;
        turnStartTime = millis();
        return 2; // Hard right turn
    }

    // Check if we should exit a hard turn when inner sensor detects line
    if (inHardLeftTurn && leftInner == 1) {
        // Exit hard left turn if inner left sensor sees the line again
        // and minimum turn time has passed
        if (millis() - turnStartTime > MIN_TURN_TIME) {
            inHardLeftTurn = false;
            lastActiveSensor = -1;
            return -1; // Switch to slight left turn
        } else {
            return -2; // Continue hard left turn until minimum time
        }
    }
    
    if (inHardRightTurn && rightInner == 1) {
        // Exit hard right turn if inner right sensor sees the line again
        // and minimum turn time has passed
        if (millis() - turnStartTime > MIN_TURN_TIME) {
            inHardRightTurn = false;
            lastActiveSensor = 1;
            return 1; // Switch to slight right turn
        } else {
            return 2; // Continue hard right turn until minimum time
        }
    }
    
    // Continue hard turns if we're in one
    if (inHardLeftTurn) {
        return -2;
    }
    if (inHardRightTurn) {
        return 2;
    }
    
    // Normal line following for slight turns
    if (leftInner == 1) {
        lastActiveSensor = -1;
        return -1; // Slight left turn
    }
    
    if (rightInner == 1) {
        lastActiveSensor = 1;
        return 1; // Slight right turn
    }
    
    // Default - go forward
    return 10; // Forward
}

void loop() {
    // Read line position
    int controlDecision = readSensors();

    // Check for finish line detection
    if (finishLineDetected) {
        stopMotors();
        
        // Don't use LED for finish line indication
        if (DEBUG_MODE && (millis() - lastLedToggle > 2000)) {
            Serial.println("Finish line detected! Car stopped.");
            lastLedToggle = millis();
        }
        return;
    }

    // Apply control decision to motors
    switch (controlDecision) {
        case 1: // Slight right turn
            setMotors(baseSpeed, turnSpeed); // Use regular turn speed instead of negative
            if (DEBUG_MODE) Serial.println("Slight right turn");
            break;
            
        case -1: // Slight left turn
            setMotors(turnSpeed, baseSpeed); // Use regular turn speed instead of negative
            if (DEBUG_MODE) Serial.println("Slight left turn");
            break;
            
        case 2: // Hard right turn
            setMotors(baseSpeed, hardTurnSpeed); // Keep baseSpeed for left motor, reverse for right
            if (DEBUG_MODE) Serial.println("Hard right turn");
            break;
            
        case -2: // Hard left turn
            setMotors(hardTurnSpeed, baseSpeed); // Reverse for left motor, keep baseSpeed for right
            if (DEBUG_MODE) Serial.println("Hard left turn");
            break;
            
        case 0: // Stop (finish line)
            stopMotors();
            finishLineDetected = true;
            if (DEBUG_MODE) Serial.println("Finish line detected! Car stopped.");
            break;
            
        case 3: // Recovery right turn
            setMotors(baseSpeed, -turnSpeed);
            if (DEBUG_MODE) Serial.println("Recovery right turn");
            break;
            
        case -3: // Recovery left turn
            setMotors(-turnSpeed, baseSpeed);
            if (DEBUG_MODE) Serial.println("Recovery left turn");
            break;
            
        default: // Forward
            setMotors(baseSpeed, baseSpeed);
            if (DEBUG_MODE) Serial.println("Going forward");
            break;
    }

    // Short delay
    delay(25); // Adjust as needed for responsiveness
}
/* STM32 Blue Pill
 * Self Driving Car - Line Following with PID Control
 * Using L298N motor driver and HW-871 line sensors
 */

#include <Arduino.h>

// Pin definitions for L298N motor driver

#define ENA PA1  // PWM control for left motors
#define ENB PA2  // PWM control for right motors
#define IN1 PA3  // Left motors direction 1
#define IN2 PA4  // Left motors direction 2
#define IN3 PA5  // Right motors direction 1
#define IN4 PA6  // Right motors direction 2

// Pin definitions for HW-871 line sensors (using B5, B6, B7, B8, B9)
#define SENSOR_1 PB3  // Leftmost sensor
#define SENSOR_2 PB4
#define SENSOR_3 PB5  // Middle sensor
#define SENSOR_4 PB6
#define SENSOR_5 PB7  // Rightmost sensor

// Define onboard LED pin
#define LED_PIN PC13  // Onboard LED on STM32 Blue Pill

// PID control constants - adjust these based on testing
float Kp = 0.5;      // Proportional gain
float Ki = 0.02;     // Integral gain
float Kd = 0.1;      // Derivative gain

// PID variables
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;

// Motor speed variables
int baseSpeed = 200;  // Base speed for motors, range 0-255

// Add debug mode for sensors
#define DEBUG_MODE true

// Calibration variables
bool calibrationMode = true;  // Start in calibration mode
int sensorMin[5] = {4095, 4095, 4095, 4095, 4095};  // Minimum sensor readings (white)
int sensorMax[5] = {0, 0, 0, 0, 0};                // Maximum sensor readings (black)
int sensorThreshold[5] = {500, 500, 500, 500, 500}; // Default threshold values
unsigned long calibrationStartTime = 0;
#define CALIBRATION_DURATION 7000  // Calibration duration in milliseconds

void setup() {
    // Initialize motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initialize line sensor pins
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);
    pinMode(SENSOR_4, INPUT);
    pinMode(SENSOR_5, INPUT);

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED is active LOW on STM32 Blue Pill

    // Start serial for debugging
    Serial.begin(115200);
    
    Serial.println("Self-Driving Car starting...");
    Serial.println("Starting sensor calibration...");
    Serial.println("Move the car over the line a few times in the next 10 seconds");
    
    // Start calibration
    calibrationStartTime = millis();
    
    // During calibration, blink LED rapidly
    while (calibrationMode) {
        // Blink LED during calibration
        digitalWrite(LED_PIN, (millis() % 200 < 100) ? LOW : HIGH);
        
        // Read and calibrate sensors
        calibrateSensors();
        
        // End calibration after time expires
        if (millis() - calibrationStartTime > CALIBRATION_DURATION) {
            calibrationMode = false;
            calculateThresholds();
            Serial.println("Calibration complete!");
            
            // Print calibration results
            Serial.println("Sensor min values (white):");
            for (int i = 0; i < 5; i++) {
                Serial.print(sensorMin[i]);
                Serial.print(" ");
            }
            Serial.println();
            
            Serial.println("Sensor max values (black):");
            for (int i = 0; i < 5; i++) {
                Serial.print(sensorMax[i]);
                Serial.print(" ");
            }
            Serial.println();
            
            Serial.println("Calculated thresholds:");
            for (int i = 0; i < 5; i++) {
                Serial.print(sensorThreshold[i]);
                Serial.print(" ");
            }
            Serial.println();
            
            delay(1000);
        }
    }
    
    digitalWrite(LED_PIN, HIGH); // Turn off LED
    delay(1000); // Short delay before starting

    // Add motor testing at startup
    Serial.println("Testing motors...");
    
    // Test left motors
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 200);
    delay(500);
    analogWrite(ENA, 0);
    
    // Test right motors
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 200);
    delay(500);
    analogWrite(ENB, 0);
    
    Serial.println("Motor test complete");
    delay(1000);
}

// Function to calculate motor speeds based on correction
void calculateMotorSpeeds(float correction, int *leftSpeed, int *rightSpeed) {
    // Base speed for both motors
    int leftBase = baseSpeed;
    int rightBase = baseSpeed;
    
    // Calculate turn ratio (0.0 to 1.0) based on correction
    // Normalize the correction value (typical range -100 to 100)
    float maxCorrection = 100.0;
    float turnRatio = constrain(abs(correction) / maxCorrection, 0.0, 1.0);
    
    // Apply non-linear scaling for more responsive turns
    // Square the turnRatio for more aggressive turning at high corrections
    float scaledTurnRatio = turnRatio * turnRatio;
    
    // Calculate minimum speed for the slower motor (for sharp turns)
    int minSpeed = baseSpeed * 0.2; // 20% of base speed for the slower wheel
    
    if (correction > 0) {
        // Need to turn right
        *leftSpeed = leftBase;
        // Scale right motor speed down based on correction
        *rightSpeed = rightBase - (rightBase - minSpeed) * scaledTurnRatio;
    } else {
        // Need to turn left
        *rightSpeed = rightBase;
        // Scale left motor speed down based on correction
        *leftSpeed = leftBase - (leftBase - minSpeed) * scaledTurnRatio;
    }
    
    if (DEBUG_MODE) {
        Serial.print("Turn ratio: ");
        Serial.print(turnRatio);
        Serial.print(" Scaled: ");
        Serial.println(scaledTurnRatio);
    }
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
    
    // Apply PWM speed control
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    
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
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Motors stopped");
}

// Function to calibrate sensors
void calibrateSensors() {
    // Read current sensor values
    int rawValues[5];
    rawValues[0] = analogRead(SENSOR_1);
    rawValues[1] = analogRead(SENSOR_2);
    rawValues[2] = analogRead(SENSOR_3);
    rawValues[3] = analogRead(SENSOR_4);
    rawValues[4] = analogRead(SENSOR_5);
    
    // Update min and max values for each sensor
    for (int i = 0; i < 5; i++) {
        // Record minimum readings (white surface)
        if (rawValues[i] < sensorMin[i]) {
            sensorMin[i] = rawValues[i];
        }
        
        // Record maximum readings (black line)
        if (rawValues[i] > sensorMax[i]) {
            sensorMax[i] = rawValues[i];
        }
    }
    
    // Print current readings every 500ms
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 500) {
        Serial.print("Current readings: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(rawValues[i]);
            Serial.print(" ");
        }
        Serial.println();
        lastPrintTime = millis();
    }
    
    delay(10); // Short delay between readings
}

// Calculate threshold values based on calibration
void calculateThresholds() {
    for (int i = 0; i < 5; i++) {
        // Set threshold at 60% between min and max (adjust as needed)
        // This bias toward detecting white can be adjusted (0.5 would be middle)
        sensorThreshold[i] = sensorMin[i] + (sensorMax[i] - sensorMin[i]) * 0.6;
        
        // Ensure we have a valid range
        if (sensorMax[i] - sensorMin[i] < 100) {
            // If the difference is too small, use default threshold
            sensorThreshold[i] = 500;
            Serial.print("Warning: Sensor ");
            Serial.print(i+1);
            Serial.println(" has a small range. Calibration may not be effective.");
        }
    }
}

// Read line position
float readLinePosition() {
    int sensorValues[5];
    int rawValues[5]; // Store raw analog values for debugging
    
    // Read sensors with analog values for better sensitivity
    rawValues[0] = analogRead(SENSOR_1);
    rawValues[1] = analogRead(SENSOR_2);
    rawValues[2] = analogRead(SENSOR_3);
    rawValues[3] = analogRead(SENSOR_4);
    rawValues[4] = analogRead(SENSOR_5);
    
    // Use calibrated threshold values for each sensor
    for (int i = 0; i < 5; i++) {
        sensorValues[i] = (rawValues[i] > sensorThreshold[i]) ? 1 : 0;
    }
    
    if (DEBUG_MODE) {
        // Print raw sensor readings for debugging
        Serial.print("Raw: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(rawValues[i]);
            Serial.print(" ");
        }
        Serial.print("| Binary: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(sensorValues[i]);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Check for finish line (all sensors reading black)
    // FIXED: Inverted the logic - 1 means black line detected
    if (sensorValues[0] == 1 && sensorValues[1] == 1 &&
        sensorValues[2] == 1 && sensorValues[3] == 1 &&
        sensorValues[4] == 1) {
        
        // Added debounce to prevent false detections
        delay(50);
        
        // Read again to confirm
        int confirmValues[5];
        confirmValues[0] = (analogRead(SENSOR_1) > sensorThreshold[0]) ? 1 : 0;
        confirmValues[1] = (analogRead(SENSOR_2) > sensorThreshold[1]) ? 1 : 0;
        confirmValues[2] = (analogRead(SENSOR_3) > sensorThreshold[2]) ? 1 : 0;
        confirmValues[3] = (analogRead(SENSOR_4) > sensorThreshold[3]) ? 1 : 0;
        confirmValues[4] = (analogRead(SENSOR_5) > sensorThreshold[4]) ? 1 : 0;
        
        // Only stop if still detecting finish line
        if (confirmValues[0] == 1 && confirmValues[1] == 1 &&
            confirmValues[2] == 1 && confirmValues[3] == 1 &&
            confirmValues[4] == 1) {
                
            stopMotors();  // Stop motors immediately
            Serial.println("Finish line detected! Stopping.");
            
            // Blink LED forever
            while (1) { 
                digitalWrite(LED_PIN, LOW);  // LED on (active LOW)
                delay(500);
                digitalWrite(LED_PIN, HIGH); // LED off
                delay(500);
            }
            
            return 999;  // Special value for finish line
        }
    }

    // Calculate weighted position
    // Invert readings: 1 for black line, 0 for white
    int sum = 0;
    int weightedSum = 0;

    for (int i = 0; i < 5; i++) {
        // FIXED: Don't invert - sensorValues already has 1 for black line
        int value = sensorValues[i];
        sum += value;
        weightedSum += value * (i * 1000);  // Position weighted by sensor position
    }

    // If no line is detected
    if (sum == 0) {
        return previousError;  // Return last known error
    }

    // Calculate position (0-4000)
    float position = weightedSum / sum;

    // Convert to error value (-2000 to +2000)
    // Middle position is 2000, so error = position - 2000
    return position - 2000;
}

// Calculate PID correction
float calculatePID(float error) {
    // Calculate time since last update
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Calculate P term
    float P = Kp * error;

    // Calculate I term
    integral += error * deltaTime;
    // Prevent integral windup
    integral = constrain(integral, -100, 100);
    float I = Ki * integral;

    // Calculate D term
    float derivative = (error - previousError) / deltaTime;
    float D = Kd * derivative;

    // Save current error for next iteration
    previousError = error;

    // Print PID values for debugging
    Serial.print("PID: Error=");
    Serial.print(error);
    Serial.print(" P=");
    Serial.print(P);
    Serial.print(" I=");
    Serial.print(I);
    Serial.print(" D=");
    Serial.println(D);

    // Return combined correction
    return P + I + D;
}

void loop() {
    // Read line position
    float error = readLinePosition();

    // Check for finish line (this is now handled in readLinePosition)
    if (error == 999) {
        return; // Already stopped in readLinePosition
    }

    // Calculate PID correction
    float correction = calculatePID(error);

    // Calculate motor speeds based on PID correction
    int leftSpeed, rightSpeed;
    calculateMotorSpeeds(correction, &leftSpeed, &rightSpeed);

    // Apply motor speeds
    setMotors(leftSpeed, rightSpeed);

    // Short delay
    delay(25); // Adjust as needed for responsiveness
}
/* ESP32
 * Self Driving Car - Line Following with PID Control
 * Using L298N motor driver, HW-871 line sensors, and HC-SR04 sonar sensor
 */

#include <Arduino.h>

// Pin definitions for L298N motor driver
#define ENA 21  // PWM control for right motors
#define ENB 19  // PWM control for left motors
#define IN1 16  // Right motors direction 1
#define IN2 17  // Right motors direction 2
#define IN3 2   // Left motors direction 1
#define IN4 4   // Left motors direction 2

// Pin definitions for HW-871 line sensors
#define SENSOR_1 26  // Leftmost sensor
#define SENSOR_2 25
#define SENSOR_3 33  // Middle sensor
#define SENSOR_4 32
#define SENSOR_5 35  // Rightmost sensor

// Pin definitions for HC-SR04 sonar sensor
#define TRIGGER_PIN 14
#define ECHO_PIN 27

// Define onboard LED pin
#define LED_PIN T2

// Define button pin for color mode toggle
#define TOGGLE_BTN 13  // Choose an available pin

// PID control constants - adjust these based on testing
float Kp = 0.38;      // Proportional gain
float Ki = 0.03;     // Integral gain
float Kd = 0.1;      // Derivative gain

// PID variables
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;

// Motor speed variables
int baseSpeed = 255;  // Base speed for motors, range 0-255

// Add debug mode for sensors
#define DEBUG_MODE true

// Line color mode (true = follow black line on white, false = follow white line on black)
bool followBlackLine = true;

// Calibration variables
bool calibrationMode = true;  // Start in calibration mode
int sensorMax[5] = {4095, 4095, 4095, 4095, 4095};  // Minimum sensor readings (white)
int sensorMin[5] = {0, 0, 0, 0, 0};                // Maximum sensor readings (black)
int sensorThreshold[5] = {1500, 1500, 1500, 1500, 1500}; // Default threshold values
unsigned long calibrationStartTime = 0;
#define CALIBRATION_DURATION 7000  // Calibration duration in milliseconds

// PWM properties for ESP32
const int freq = 5000;
const int resolution = 8;  // 8-bit resolution, 0-255

// Sonar variables
const int maxDistance = 200; // Maximum distance in cm
long duration, distance;     // Variables for sonar measurements

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    Serial.println("ESP32 Self-Driving Car starting...");
    
    // Configure LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Turn off LED (may be inverted on some boards)
    
    // Configure motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Configure ESP32 PWM for motor control using the new API (ESP32 v3.x)
    ledcAttach(ENA, freq, resolution);
    ledcAttach(ENB, freq, resolution);
    
    // Initialize line sensor pins
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);
    pinMode(SENSOR_4, INPUT);
    pinMode(SENSOR_5, INPUT);
    
    // Initialize sonar sensor pins
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Initialize button pin for color mode toggle
    pinMode(TOGGLE_BTN, INPUT_PULLUP);
    
    // Check toggle button during startup to set line tracking mode
    if (digitalRead(TOGGLE_BTN) == LOW) {
        // Button pressed during startup - toggle line following mode
        followBlackLine = !followBlackLine;
        
        // Indicate which mode is active with LED blinks
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, LOW);  // LED on
            delay(200);
            digitalWrite(LED_PIN, HIGH); // LED off
            delay(200);
        }
        
        Serial.print("Line tracking mode set to follow ");
        Serial.println(followBlackLine ? "BLACK line" : "WHITE line");
        
        // Wait for button release
        while (digitalRead(TOGGLE_BTN) == LOW) {
            delay(10);
        }
        delay(500); // Debounce
    }
    
    Serial.println("Starting sensor calibration...");
    Serial.println("Move the car over the line a few times in the next 7 seconds");
    
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

    // Test motors
    Serial.println("Testing motors...");
    
    // Test left motors
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 200);
    delay(500);
    ledcWrite(ENA, 0);
    
    // Test right motors
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 200);
    delay(500);
    ledcWrite(ENB, 0);
    
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
    
    // Apply PWM speed control using ESP32's new ledcWrite API (directly to pins)
    ledcWrite(ENA, leftSpeed);
    ledcWrite(ENB, rightSpeed);
    
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
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
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
    
    // Apply thresholds based on the line detection mode
    for (int i = 0; i < 5; i++) {
        if (followBlackLine) {
            // Black line mode: 1 when value > threshold (black detected)
            sensorValues[i] = (rawValues[i] > sensorThreshold[i]) ? 1 : 0;
        } else {
            // White line mode: 1 when value < threshold (white detected)
            sensorValues[i] = (rawValues[i] < sensorThreshold[i]) ? 1 : 0;
        }
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
        Serial.print("| Mode: ");
        Serial.println(followBlackLine ? "BLACK" : "WHITE");
    }

    // Check for finish line (all sensors reading the same)
    bool allSame = true;
    for (int i = 1; i < 5; i++) {
        if (sensorValues[i] != sensorValues[0]) {
            allSame = false;
            break;
        }
    }
    
    // Finish line is when all sensors detect line (all 1s)
    if (allSame && sensorValues[0] == 1) {
        // Added debounce to prevent false detections
        delay(50);
        
        // Read again to confirm
        int confirmValues[5];
        for (int i = 0; i < 5; i++) {
            int rawValue = analogRead(i == 0 ? SENSOR_1 : 
                                    i == 1 ? SENSOR_2 : 
                                    i == 2 ? SENSOR_3 : 
                                    i == 3 ? SENSOR_4 : SENSOR_5);
                                    
            if (followBlackLine) {
                confirmValues[i] = (rawValue > sensorThreshold[i]) ? 1 : 0;
            } else {
                confirmValues[i] = (rawValue < sensorThreshold[i]) ? 1 : 0;
            }
        }
        
        // Check if still all 1s
        bool stillAllSame = true;
        for (int i = 1; i < 5; i++) {
            if (confirmValues[i] != confirmValues[0] || confirmValues[0] != 1) {
                stillAllSame = false;
                break;
            }
        }
        
        // Only stop if still detecting finish line
        if (stillAllSame) {
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
    int sum = 0;
    int weightedSum = 0;

    for (int i = 0; i < 5; i++) {
        sum += sensorValues[i];
        weightedSum += sensorValues[i] * (i * 1000);  // Position weighted by sensor position
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

// Function to measure distance using HC-SR04 sonar sensor
int measureDistance() {
    // Clear the trigger pin
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    
    // Send 10μs pulse to trigger
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    // Read echo pin (duration in microseconds)
    duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
    
    // Calculate distance in centimeters
    // Speed of sound is ~343m/s = 0.0343cm/μs
    // Distance = time * speed / 2 (round-trip)
    distance = (duration * 0.0343) / 2;
    
    // Check for timeout or invalid reading
    if (duration == 0 || distance > maxDistance || distance <= 0) {
        distance = maxDistance; // Default to max distance on error
    }
    
    if (DEBUG_MODE) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
    
    return distance;
}

void loop() {
    // Measure distance to any obstacles in front
    int obstacleDistance = measureDistance();
    
    // Check if there's an obstacle too close
    if (obstacleDistance < 20) { // Less than 20cm
        // Stop car
        stopMotors();
        Serial.print("Obstacle detected at ");
        Serial.print(obstacleDistance);
        Serial.println("cm. Stopping.");
        
        // Wait briefly
        delay(300);
        
        // Backup slightly
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(ENA, 180);
        ledcWrite(ENB, 180);
        delay(400);
        stopMotors();
        
        // Turn to avoid obstacle (turn right by default)
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(ENA, 200);
        ledcWrite(ENB, 200);
        delay(500); // Adjust turn duration as needed
        stopMotors();
        
        return; // Return to beginning of loop to reassess
    }

    // Read line position
    float error = readLinePosition();

    // Check for finish line (this is handled in readLinePosition)
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
    delay(20); // Adjust as needed for responsiveness
}
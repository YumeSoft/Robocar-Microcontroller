/*
 * RC Car with Arduino UNO and L298N Motor Controller
 * - One motor for driving (propulsion)
 * - One motor for steering
 */

// Pin definitions for L298N motor controller
// Drive motor
#define DRIVE_ENA 5  // Enable A - PWM speed control for drive motor
#define DRIVE_IN1 7  // Input 1 for drive motor direction
#define DRIVE_IN2 8  // Input 2 for drive motor direction

// Steering motor
#define STEER_ENB 6  // Enable B - PWM speed control for steering motor
#define STEER_IN3 9  // Input 3 for steering motor direction
#define STEER_IN4 10 // Input 4 for steering motor direction

// RC Receiver pin definitions
#define RC_FORWARD A0  // Forward signal from RC receiver
#define RC_BACKWARD A1 // Backward signal from RC receiver
#define RC_LEFT A2     // Left signal from RC receiver
#define RC_RIGHT A3    // Right signal from RC receiver

// Threshold for input detection - adjust based on your receiver
#define RC_THRESHOLD 500  // Typical threshold for analog reading

// Variables for motor control
int driveSpeed = 0;  // Speed for drive motor (0-255)
int steerPosition = 0; // Current steering position

void setup() {
    // Initialize motor control pins
    pinMode(DRIVE_ENA, OUTPUT);
    pinMode(DRIVE_IN1, OUTPUT);
    pinMode(DRIVE_IN2, OUTPUT);
    
    pinMode(STEER_ENB, OUTPUT);
    pinMode(STEER_IN3, OUTPUT);
    pinMode(STEER_IN4, OUTPUT);
    
    // Initialize RC receiver pins as inputs
    pinMode(RC_FORWARD, INPUT);
    pinMode(RC_BACKWARD, INPUT);
    pinMode(RC_LEFT, INPUT);
    pinMode(RC_RIGHT, INPUT);
    
    // Start serial for debugging and commands
    Serial.begin(9600);
    Serial.println("RC Car initialized");
    
    // Ensure motors are stopped at startup
    stopDrive();
    centerSteering();
}

void loop() {
    // Check for commands from Serial Monitor (for testing)
    if (Serial.available() > 0) {
        char command = Serial.read();
        processCommand(command);
    }
    
    // Read RC receiver inputs
    readRCInputs();
}

// Read and process RC receiver inputs
void readRCInputs() {
    // Read signals from RC receiver
    int forwardSignal = analogRead(RC_FORWARD);
    int backwardSignal = analogRead(RC_BACKWARD);
    int leftSignal = analogRead(RC_LEFT);
    int rightSignal = analogRead(RC_RIGHT);
    
    // Debug - uncomment to see the values
    // Serial.println("F:" + String(forwardSignal) + " B:" + String(backwardSignal) + 
    //                " L:" + String(leftSignal) + " R:" + String(rightSignal));
    
    // Process forward/backward movement
    if (forwardSignal > RC_THRESHOLD) {
        // Map the signal strength to motor speed (adjust mapping as needed)
        int speed = map(forwardSignal, RC_THRESHOLD, 1023, 100, 255);
        driveForward(speed);
    } 
    else if (backwardSignal > RC_THRESHOLD) {
        int speed = map(backwardSignal, RC_THRESHOLD, 1023, 100, 255);
        driveBackward(speed);
    }
    else {
        stopDrive();
    }
    
    // Process left/right steering
    if (leftSignal > RC_THRESHOLD) {
        int speed = map(leftSignal, RC_THRESHOLD, 1023, 100, 255);
        steerLeft(speed);
    }
    else if (rightSignal > RC_THRESHOLD) {
        int speed = map(rightSignal, RC_THRESHOLD, 1023, 100, 255);
        steerRight(speed);
    }
    else {
        centerSteering();
    }
    
    // Add a small delay to prevent readings from being too rapid
    delay(50);
}

void processCommand(char cmd) {
    switch (cmd) {
        case 'F': // Forward
            driveForward(200);
            break;
        case 'B': // Backward
            driveBackward(200);
            break;
        case 'L': // Left
            steerLeft(200);
            break;
        case 'R': // Right
            steerRight(200);
            break;
        case 'S': // Stop
            stopDrive();
            break;
        case 'C': // Center steering
            centerSteering();
            break;
    }
}

// Drive motor functions
void driveForward(int speed) {
    digitalWrite(DRIVE_IN1, HIGH);
    digitalWrite(DRIVE_IN2, LOW);
    analogWrite(DRIVE_ENA, speed);
    Serial.println("Driving forward");
}

void driveBackward(int speed) {
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, HIGH);
    analogWrite(DRIVE_ENA, speed);
    Serial.println("Driving backward");
}

void stopDrive() {
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, LOW);
    analogWrite(DRIVE_ENA, 0);
    Serial.println("Drive stopped");
}

// Steering motor functions
void steerLeft(int speed) {
    digitalWrite(STEER_IN3, HIGH);
    digitalWrite(STEER_IN4, LOW);
    analogWrite(STEER_ENB, speed);
    Serial.println("Steering left");
    // In a real application, you might want to limit steering time
    // or use position feedback to prevent over-rotation
}

void steerRight(int speed) {
    digitalWrite(STEER_IN3, LOW);
    digitalWrite(STEER_IN4, HIGH);
    analogWrite(STEER_ENB, speed);
    Serial.println("Steering right");
    // In a real application, you might want to limit steering time
    // or use position feedback to prevent over-rotation
}

void centerSteering() {
    // Stop the steering motor
    digitalWrite(STEER_IN3, LOW);
    digitalWrite(STEER_IN4, LOW);
    analogWrite(STEER_ENB, 0);
    Serial.println("Steering centered");
    
    // Note: For accurate centering, you would typically need
    // position feedback (like a potentiometer or limit switches)
}
// Self-Driving Car with Line Following Capabilities
// Using LN298N Motor Driver and HW871 Line Reader

// Motor Driver Pins
#define ENA 10
#define ENB 11
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// Line Sensor Pins
#define SENSOR1 A1 // Left most
#define SENSOR2 A2 // Left middle
#define SENSOR3 A3 // Middle
#define SENSOR4 A4 // Right middle
#define SENSOR5 A5 // Right most

// Constants for line detection
#define BLACK_LINE 1  // 1 for black line, 0 for white line
#define CALIBRATION_TIME 7000 // 7 seconds for calibration
#define START_DELAY 2000 // 2 seconds before starting
#define ALL_BLACK_THRESHOLD 100 // Threshold to determine if all sensors see black

// LED Status Indicators
#define LED_PIN LED_BUILTIN
#define LED_BLINK_NORMAL 500    // Normal operation - blink every 500ms
#define LED_BLINK_LOST 100      // Lost line - blink fast every 100ms
#define LED_BLINK_STOPPED 250   // All black detected - blink pattern

// Motor speed settings
#define BASE_SPEED 160
#define SOFT_TURN_SPEED 140
#define MED_TURN_SPEED 120
#define HARD_TURN_SPEED 100

// Variables for line sensor calibration
int minValues[5] = {4095, 4095, 4095, 4095, 4095};
int maxValues[5] = {0, 0, 0, 0, 0};
int threshold[5] = {0, 0, 0, 0, 0};

// Variables for line following logic
int sensorValues[5] = {0, 0, 0, 0, 0};
int sensorOnLine[5] = {0, 0, 0, 0, 0}; // 1 if sensor is on line, 0 otherwise
int lastPosition = 0; // 0 for left, 1 for right

// LED status variables
unsigned long lastLedUpdate = 0;
bool ledState = false;
int currentMode = 0;  // 0: normal, 1: lost line, 2: stopped

// PID Control Variables
float Kp = 25; // Proportional gain
float Ki = 0; // Integral gain
float Kd = 10; // Derivative gain
int error = 0;
int previousError = 0;
int integral = 0;
int derivative = 0;
int PIDvalue = 0;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  
  // Setup Motor Driver Pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  
  // Stop the motors initially
  motorStop();
  
  // Calibration Phase
  Serial.println("Calibrating sensors...");
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED during calibration
  
  long startTime = millis();
  while (millis() - startTime < CALIBRATION_TIME) {
    // Read sensors and update min/max values
    for (int i = 0; i < 5; i++) {
      int sensorValue = analogRead(A1 + i);
      if (sensorValue < minValues[i]) {
        minValues[i] = sensorValue;
      }
      if (sensorValue > maxValues[i]) {
        maxValues[i] = sensorValue;
      }
    }
    delay(10);
  }
  
  // Calculate threshold values
  for (int i = 0; i < 5; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" - Min: ");
    Serial.print(minValues[i]);
    Serial.print(" Max: ");
    Serial.print(maxValues[i]);
    Serial.print(" Threshold: ");
    Serial.println(threshold[i]);
  }
  
  digitalWrite(LED_BUILTIN, LOW); // Turn off calibration LED
  Serial.println("Calibration complete!");
  
  // Wait before starting
  Serial.print("Starting in ");
  for (int i = START_DELAY/1000; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println("GO!");
}

void loop() {
  // Read and normalize sensor values
  readSensors();
  
  // Check if all sensors see black (stop condition)
  if (checkAllBlack()) {
    motorStop();
    Serial.println("All black detected - STOP");
    currentMode = 2; // Stopped mode
    updateLedStatus(); // Update LED immediately
    delay(1000); // Wait before checking again
    return;
  }
  
  // Simple turning logic
  simpleTurnLogic();
  
  /* 
  // Alternate method: Use PID control for line following
  // Uncomment this section and comment out simpleTurnLogic() to use PID
  calculatePID();
  int leftMotorSpeed = BASE_SPEED - PIDvalue;
  int rightMotorSpeed = BASE_SPEED + PIDvalue;
  
  // Constrain motor speeds
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  
  // Drive motors
  motorDrive(leftMotorSpeed, rightMotorSpeed);
  */
  
  // Update LED status based on current mode
  updateLedStatus();
  
  delay(10); // Small delay for stability
}

// Update LED based on current operating mode
void updateLedStatus() {
  unsigned long currentTime = millis();
  
  switch (currentMode) {
    case 0: // Normal operation - slow blink
      if (currentTime - lastLedUpdate >= LED_BLINK_NORMAL) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastLedUpdate = currentTime;
      }
      break;
      
    case 1: // Lost line - fast blink
      if (currentTime - lastLedUpdate >= LED_BLINK_LOST) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastLedUpdate = currentTime;
      }
      break;
      
    case 2: // Stopped - double blink pattern
      int cyclePosition = (currentTime / 250) % 4;
      if (cyclePosition == 0 || cyclePosition == 1) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      break;
  }
}

// Read sensors and determine if they're on the line
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(A1 + i);
    
    // Detect if sensor is on line based on BLACK_LINE setting
    if (BLACK_LINE) {
      // For black line, higher values mean on line
      sensorOnLine[i] = (sensorValues[i] > threshold[i]) ? 1 : 0;
    } else {
      // For white line, lower values mean on line
      sensorOnLine[i] = (sensorValues[i] < threshold[i]) ? 1 : 0;
    }
  }
  
  // Debug output
  Serial.print("Sensors: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensorOnLine[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// Check if all sensors detect black
bool checkAllBlack() {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += sensorOnLine[i];
  }
  return (sum >= ALL_BLACK_THRESHOLD);
}

// Simple turn logic as specified in requirements
void simpleTurnLogic() {
  // Left most sensor is S0, Middle is S2, Right most is S4
  
  // Hard right turn - leftmost sensor on line
  if (sensorOnLine[0] == 1 && sensorOnLine[1] == 0 && sensorOnLine[2] == 0 && sensorOnLine[3] == 0 && sensorOnLine[4] == 0) {
    motorDrive(HARD_TURN_SPEED, BASE_SPEED);
    lastPosition = 0; // Remember last position was left
    currentMode = 0;  // Normal operation
  }
  // Medium right turn - left middle sensor on line
  else if (sensorOnLine[0] == 0 && sensorOnLine[1] == 1 && sensorOnLine[2] == 0 && sensorOnLine[3] == 0 && sensorOnLine[4] == 0) {
    motorDrive(MED_TURN_SPEED, BASE_SPEED);
    lastPosition = 0; // Remember last position was left
    currentMode = 0;  // Normal operation
  }
  // Soft right turn - left middle and center sensors on line
  else if (sensorOnLine[0] == 0 && sensorOnLine[1] == 1 && sensorOnLine[2] == 1 && sensorOnLine[3] == 0 && sensorOnLine[4] == 0) {
    motorDrive(SOFT_TURN_SPEED, BASE_SPEED);
    currentMode = 0;  // Normal operation
  }
  // Hard left turn - rightmost sensor on line
  else if (sensorOnLine[0] == 0 && sensorOnLine[1] == 0 && sensorOnLine[2] == 0 && sensorOnLine[3] == 0 && sensorOnLine[4] == 1) {
    motorDrive(BASE_SPEED, HARD_TURN_SPEED);
    lastPosition = 1; // Remember last position was right
    currentMode = 0;  // Normal operation
  }
  // Medium left turn - right middle sensor on line
  else if (sensorOnLine[0] == 0 && sensorOnLine[1] == 0 && sensorOnLine[2] == 0 && sensorOnLine[3] == 1 && sensorOnLine[4] == 0) {
    motorDrive(BASE_SPEED, MED_TURN_SPEED);
    lastPosition = 1; // Remember last position was right
    currentMode = 0;  // Normal operation
  }
  // Soft left turn - right middle and center sensors on line
  else if (sensorOnLine[0] == 0 && sensorOnLine[1] == 0 && sensorOnLine[2] == 1 && sensorOnLine[3] == 1 && sensorOnLine[4] == 0) {
    motorDrive(BASE_SPEED, SOFT_TURN_SPEED);
    currentMode = 0;  // Normal operation
  }
  // Straight ahead - center sensor on line
  else if (sensorOnLine[2] == 1 && sensorOnLine[0] == 0 && sensorOnLine[4] == 0) {
    motorDrive(BASE_SPEED, BASE_SPEED);
    currentMode = 0;  // Normal operation
  }
  // Lost line - use last known position
  else if (sensorOnLine[0] == 0 && sensorOnLine[1] == 0 && sensorOnLine[2] == 0 && sensorOnLine[3] == 0 && sensorOnLine[4] == 0) {
    if (lastPosition == 0) { // Last known position was left
      motorDrive(HARD_TURN_SPEED, BASE_SPEED); // Turn right to find line
    } else {
      motorDrive(BASE_SPEED, HARD_TURN_SPEED); // Turn left to find line
    }
    currentMode = 1;  // Lost line mode
  }
  // Default - go straight
  else {
    motorDrive(BASE_SPEED, BASE_SPEED);
    currentMode = 0;  // Normal operation
  }
}

// PID calculation for line following
void calculatePID() {
  // Calculate weighted position
  int position = 0;
  int sum = 0;
  
  for (int i = 0; i < 5; i++) {
    position += sensorOnLine[i] * i * 1000;
    sum += sensorOnLine[i];
  }
  
  // Avoid division by zero
  if (sum > 0) {
    position = position / sum;
  }
  
  // Calculate error from center position (2000 for 5 sensors)
  error = position - 2000;
  
  // Calculate PID components
  integral = integral + error;
  derivative = error - previousError;
  
  // Calculate final PID value
  PIDvalue = (Kp * error + Ki * integral + Kd * derivative) / 100;
  
  // Remember current error for next iteration
  previousError = error;
  
  // Constrain PID value
  PIDvalue = constrain(PIDvalue, -255, 255);
  
  // Debug output
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print(" PID: ");
  Serial.println(PIDvalue);
}

// Control both motors
void motorDrive(int leftSpeed, int rightSpeed) {
  // Left Motor control (IN1, IN2)
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Right Motor control (IN3, IN4)
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Debug output
  Serial.print("Motors: L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
}

// Stop both motors
void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Motors stopped");
}
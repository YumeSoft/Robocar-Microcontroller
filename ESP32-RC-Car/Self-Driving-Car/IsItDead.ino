void setup(){
    // Initialize LED pin
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);  // LED is active LOW on STM32 Blue Pill
}

void loop(){
    // Blink LED to indicate the program is running
    digitalWrite(PC13, LOW);  // LED on
    delay(500);
    digitalWrite(PC13, HIGH); // LED off
    delay(500);
}
/*
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Motor Control Pins
#define ENA 33
#define IN1 26
#define IN2 27

// ToF Sensor
#define LOX1_ADDRESS 0x30
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();

int sensorReading = 0; 
volatile int motorState = 0; // Shared between tasks
unsigned long lastSensorRead = 0;

// Sensor Watchdog (to detect if sensor stops responding)
unsigned long lastValidSensorTime = 0;
const unsigned long SENSOR_TIMEOUT = 1000; // 1 second

// Function to safely read ToF Sensor
int readSensor() {
    int distance = lox1.readRange();
    if (lox1.timeoutOccurred()) {
        Serial.println("Sensor error! Reinitializing...");
        lox1.begin(LOX1_ADDRESS);
        lox1.startRangeContinuous();
    }
    return distance;
}

// Task to read sensor continuously
void sensorTask(void *parameter) {
    while (1) {
        if (millis() - lastSensorRead >= 50) { // Read every 50ms
            sensorReading = readSensor();

            // Check if valid data received
            if (sensorReading > 0 && sensorReading < 2000) {
                lastValidSensorTime = millis();
            }

            Serial.print("Sensor Reading: ");
            Serial.println(sensorReading);
            lastSensorRead = millis();
        }

        // Sensor recovery logic
        if (millis() - lastValidSensorTime > SENSOR_TIMEOUT) {
            Serial.println("Sensor timeout! Reinitializing...");
            lox1.begin(LOX1_ADDRESS);
            lox1.startRangeContinuous();
            lastValidSensorTime = millis(); // Prevent continuous resets
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Task to control motor based on user input
void motorTask(void *parameter) {
    while (1) {
        if (motorState == 1) {
            // Move forward
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            // analogWrite(ENA, 150); // PWM control commented out
            digitalWrite(ENA, HIGH); // Enable motor at full speed
        } 
        else if (motorState == 2) {
            // Move backward
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            // analogWrite(ENA, 150); // PWM control commented out
            digitalWrite(ENA, HIGH); // Enable motor at full speed
        } 
        else {
            // Stop
            // analogWrite(ENA, 0); // PWM control commented out
            digitalWrite(ENA, LOW); // Disable motor
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Initialize ToF Sensor
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to boot VL53L0X");
        while (1);
    }
    lox1.startRangeContinuous();
    lastValidSensorTime = millis(); // Initialize watchdog timer

    // Create tasks with different priorities
    xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 2, NULL); // Higher priority
    xTaskCreate(motorTask, "Motor Task", 4096, NULL, 1, NULL);   // Lower priority
}

void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        Serial.flush(); // Clear input buffer

        if (input == '1') {
            motorState = 1; // Move forward
        } 
        else if (input == '2') {
            motorState = 2; // Move backward
        } 
        else {
            motorState = 0; // Stop
        }

        Serial.print("Motor State: ");
        Serial.println(motorState);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
*/
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Motor Control Pins
#define ENA 33
#define IN1 26
#define IN2 27

// ToF Sensor
#define LOX1_ADDRESS 0x30
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();

void setup() {
    Serial.begin(115200);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Initialize ToF Sensor
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to boot VL53L0X");
        while (1);
    }
    lox1.startRangeContinuous();
}

void loop() {
    // Move motor forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(ENA, HIGH);
    Serial.println("Motor Forward");
    delay(2000);

    // Read ToF sensor value
    int distance = lox1.readRange();
    Serial.print("Distance: ");
    Serial.println(distance);
    delay(1000);

    // Move motor backward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    Serial.println("Motor Backward");
    delay(2000);

    // Read ToF sensor value again
    distance = lox1.readRange();
    Serial.print("Distance: ");
    Serial.println(distance);
    delay(1000);
}


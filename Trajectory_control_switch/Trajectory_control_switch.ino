#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Define Actuator Pins
#define ENA 33
#define IN1 26
#define IN2 27
#define ENB 12
#define IN3 14
#define IN4 32

// Define ToF Sensor Addresses and Shutdown Pins
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define SHT_LOX1 4
#define SHT_LOX2 5

// VL53L0X Objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Feedback Loop Variables
#define MOVING_AVG_SIZE 5
#define PROPORTIONAL_GAIN 1  

int baseline1 = 0, baseline2 = 0;
bool isCalibrated = false; // Persistent flag
int buffer1[MOVING_AVG_SIZE] = {0}, buffer2[MOVING_AVG_SIZE] = {0};
int index1 = 0, index2 = 0;
long sum1 = 0, sum2 = 0;
int targetPositions1[2] = {10, 45};
int targetPositions2[2] = {30, 45};
int targetIndex1 = 0, targetIndex2 = 0;

// Selected Mode
int mode = 0;  

// Sensor Reinitialization Variables
unsigned long lastValidReadTime = 0;
const unsigned long SENSOR_TIMEOUT = 2000; // 2 seconds timeout

void moveActuators(int speed, bool forward) {
    if (forward) {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    }
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void moveActuatorsToFeedback(int ActuatorSpeed, int avg1, int avg2) {
    int error1 = targetPositions1[targetIndex1] - avg1;
    int error2 = targetPositions2[targetIndex2] - avg2;

    digitalWrite(IN1, error1 > 0 ? LOW : HIGH);
    digitalWrite(IN2, error1 > 0 ? HIGH : LOW);
    digitalWrite(IN3, error2 > 0 ? HIGH : LOW);
    digitalWrite(IN4, error2 > 0 ? LOW : HIGH);

    analogWrite(ENA, ActuatorSpeed);
    analogWrite(ENB, ActuatorSpeed);

    if (abs(error1) < 1 && abs(error2) < 5) {
        targetIndex1 = (targetIndex1 + 1) % 2;
        targetIndex2 = (targetIndex2 + 1) % 2;
        Serial.println("Both actuators reached targets. Moving to next targets.");
    }
}

void setID() {
    digitalWrite(SHT_LOX1, LOW); digitalWrite(SHT_LOX2, LOW); delay(10);
    digitalWrite(SHT_LOX1, HIGH); digitalWrite(SHT_LOX2, HIGH); delay(10);

    digitalWrite(SHT_LOX1, HIGH); digitalWrite(SHT_LOX2, LOW);
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to boot first VL53L0X"); while (1);
    }

    digitalWrite(SHT_LOX2, HIGH); delay(10);
    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println("Failed to boot second VL53L0X"); while (1);
    }
}

void reinitializeSensors() {
    
    setup();
    /*
    Serial.println("Reinitializing Sensors...");
    setID();
    lox1.startRangeContinuous();
    lox2.startRangeContinuous();
    Serial.println("Sensors Restarted Successfully.");
    */
}

void read_dual_sensors() {
    int raw1 = -1, raw2 = -1;

    if (lox1.isRangeComplete()) {
        raw1 = lox1.readRange();
    }

    if (lox2.isRangeComplete()) {
        raw2 = lox2.readRange();
    }

    // Check if valid readings received
    if (raw1 != -1 || raw2 != -1) {
        lastValidReadTime = millis(); // Update last valid read time
    }

    // If no valid data for 2 seconds, reset sensors
    if (millis() - lastValidReadTime > SENSOR_TIMEOUT) {
        reinitializeSensors();
        lastValidReadTime = millis(); // Prevent immediate re-reset
        return;
    }

    if (raw1 == -1) raw1 = baseline1; // Use last known good value
    if (raw2 == -1) raw2 = baseline2;

    // Only calibrate ONCE at the start
    if (!isCalibrated) {
        baseline1 = raw1; 
        baseline2 = raw2;
        isCalibrated = true; // Set flag to avoid future recalibration
        Serial.println("Calibration done!");
    }

    int relative1 = raw1 - baseline1;
    int relative2 = raw2 - baseline2;

    sum1 -= buffer1[index1]; sum2 -= buffer2[index2];
    buffer1[index1] = relative1; buffer2[index2] = relative2;
    sum1 += buffer1[index1]; sum2 += buffer2[index2];
    index1 = (index1 + 1) % MOVING_AVG_SIZE;
    index2 = (index2 + 1) % MOVING_AVG_SIZE;

    int avg1 = sum1 / MOVING_AVG_SIZE;
    int avg2 = sum2 / MOVING_AVG_SIZE;

    Serial.print("Sensor1: "); Serial.print(avg1);
    Serial.print("  Sensor2: "); Serial.println(avg2);

    if (mode == 4) moveActuatorsToFeedback(100, avg1, avg2);
}

void setup() {
    Serial.begin(115200);
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(SHT_LOX1, OUTPUT); pinMode(SHT_LOX2, OUTPUT);

    Serial.println("Initializing sensors...");
    digitalWrite(SHT_LOX1, LOW); digitalWrite(SHT_LOX2, LOW);
    setID();
    Serial.println("Enter mode: 1 = Move Forward, 2 = Move Backward, 3 = Read Sensors, 4 = Feedback Control");

    lox1.startRangeContinuous();
    lox2.startRangeContinuous();
    lastValidReadTime = millis(); // Start timeout tracking
}

void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        if (input >= '1' && input <= '4') {
            mode = input - '0';
            Serial.print("Selected mode: "); Serial.println(mode);
        }
    }

    switch (mode) {
        case 1: 
            moveActuators(100, true);
            Serial.println("Actuators Moving Forward"); 
            break;
        case 2: 
            moveActuators(100, false);
            Serial.println("Actuators Moving Backward"); 
            break;
        case 3: 
            read_dual_sensors(); 
            break;
        case 4: 
            read_dual_sensors(); 
            break;
        default: 
            Serial.println("Waiting for input..."); 
    }
    
    delay(50); // Prevents CPU overload
}

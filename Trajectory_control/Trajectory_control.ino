#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Address for dual sensor setup
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Shutdown pins
#define SHT_LOX1 4
#define SHT_LOX2 5

// Define pins for Actuators
#define ENA 33
#define IN1 26
#define IN2 27
#define ENB 12
#define IN3 14
#define IN4 32

// Number of samples for moving average
#define MOVING_AVG_SIZE 5

// VL53L0X objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Measurement data
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// Baseline (first reading)
int baseline1 = 0, baseline2 = 0;
bool isCalibrated = false;

// Moving average buffers
int buffer1[MOVING_AVG_SIZE] = {0};
int buffer2[MOVING_AVG_SIZE] = {0};
int index1 = 0, index2 = 0;
long sum1 = 0, sum2 = 0;

// Set of target positions for actuator 1 (in mm)

int targetPositions1[] = {163, 169, 173, 173, 176, 159, 151,142,163};
int targetPositions2[] = {112, 97, 85, 76, 72, 73, 79, 89, 102,111};

int targetIndex1 = 0; // Index for target positions of actuator 1
int targetIndex2 = 0; // Index for target positions of actuator 2

// Actuator control constants
#define PROPORTIONAL_GAIN 1  // Proportional gain for control

void setID() {
    // Reset all sensors
    digitalWrite(SHT_LOX1, LOW);    
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    // Bring sensors out of reset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // Activate LOX1 and reset LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    // Initialize LOX1
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println(F("Failed to boot first VL53L0X"));
        while (1);
    }
    delay(10);

    // Activate LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // Initialize LOX2
    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1);
    }
}

void read_dual_sensors() {
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);

    int raw1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : baseline1;
    int raw2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : baseline2;

    // Calibration: Set first reading as zero
    if (!isCalibrated) {
        baseline1 = raw1;
        baseline2 = raw2;
        isCalibrated = true;
        Serial.println(F("Calibration done! Using first values as zero reference."));
    }

    // Calculate relative values
    int relative1 = raw1 - baseline1;
    int relative2 = raw2 - baseline2;

    // Apply moving average
    sum1 -= buffer1[index1];   // Remove oldest value from sum
    sum2 -= buffer2[index2];
    
    buffer1[index1] = relative1; // Add new value
    buffer2[index2] = relative2;
    
    sum1 += buffer1[index1];  // Update sum with new value
    sum2 += buffer2[index2];

    index1 = (index1 + 1) % MOVING_AVG_SIZE;  // Increment index (circular buffer)
    index2 = (index2 + 1) % MOVING_AVG_SIZE;

    int avg1 = sum1 / MOVING_AVG_SIZE;
    int avg2 = sum2 / MOVING_AVG_SIZE;

    // Print results

    // Print results in a format readable by Serial Plotter
    //Serial.print("Sensor1: ");
    Serial.println(avg1);
    Serial.print("\t");  // Use tab to separate values
    //Serial.println("Sensor2: ");
    Serial.println(avg2); // New line for the next data set


    // Move actuators based on feedback from sensors
    moveActuators(100, avg1, avg2);

    delay(100);
}

// Function to move actuators based on sensor feedback
void moveActuators(int ActuatorSpeed,int avg1, int avg2) {
    // Actuator 1 control
    int error1 = targetPositions1[targetIndex1] - avg1;
    int controlSignal1 = PROPORTIONAL_GAIN * error1;

    // Move actuator 1
    if (controlSignal1 > 0) {
        // Move forward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, ActuatorSpeed);
    } else {
        // Move backward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        
         analogWrite(ENA, ActuatorSpeed);
    }

    // Actuator 2 control
    int error2 = targetPositions2[targetIndex2] - avg2;
    int controlSignal2 = PROPORTIONAL_GAIN * error2;

    // Move actuator 2
    if (controlSignal2 > 0) {
        // Move forward
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, ActuatorSpeed);
    } else {
        // Move backward
        
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, ActuatorSpeed);
    }

 // If both actuators have reached their targets, move to the next targets
if (abs(error1) < 1 && abs(error2) < 5) {
    targetIndex1 = (targetIndex1 + 1) % 10;
    targetIndex2 = (targetIndex2 + 1) % 10;

    Serial.println(F("Both actuators reached their targets. Moving to next targets."));
    
    Serial.print(F("Actuator 1 new target: "));
    Serial.println(targetPositions1[targetIndex1]);

    Serial.print(F("Actuator 2 new target: "));
    Serial.println(targetPositions2[targetIndex2]);
}

}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(1); }

    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    

    Serial.println(F("Initializing sensors..."));
    
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    
    Serial.println(F("Resetting sensors..."));
    
    setID();
}

void loop() {
    read_dual_sensors();
    
}

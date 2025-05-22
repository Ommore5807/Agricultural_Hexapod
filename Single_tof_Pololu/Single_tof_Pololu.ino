
#include <Wire.h>
#include <VL53L0X.h>

// Define sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

// Define sensor reset pins
#define SENSOR1_SDA 21
#define SENSOR1_SCL 22
#define SENSOR1_RESET 4

#define SENSOR2_SDA 18
#define SENSOR2_SCL 19
#define SENSOR2_RESET 5

void initSensor(VL53L0X &sensor, uint8_t sdaPin, uint8_t sclPin, uint8_t resetPin) {
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, HIGH);  // Keep high to enable sensor

    Wire.begin(sdaPin, sclPin);
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1);
    }

    sensor.setMeasurementTimingBudget(20000);  // Adjust timing budget
    sensor.startContinuous();
}

void resetSensor(VL53L0X &sensor, uint8_t resetPin) {
    Serial.println("Resetting Sensor...");
    sensor.stopContinuous();
    Wire.endTransmission();

    digitalWrite(resetPin, LOW);
    delay(600);
    digitalWrite(resetPin, HIGH);
    delay(600);

    if (!sensor.init()) {
        Serial.println("Sensor failed to restart!");
        ESP.restart();
    }
    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous();
}

void readSensor(VL53L0X &sensor, uint8_t resetPin, uint16_t *data) {
    static uint8_t errorCounter = 0;

    *data = sensor.readRangeContinuousMillimeters();

    if (*data >= 1000) {
        errorCounter++;
        if (errorCounter == 10) {
            resetSensor(sensor, resetPin);
            errorCounter = 0;
        }
    } else {
        errorCounter = 0;
    }

    Serial.println(*data);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize both sensors
    initSensor(sensor1, SENSOR1_SDA, SENSOR1_SCL, SENSOR1_RESET);
    initSensor(sensor2, SENSOR2_SDA, SENSOR2_SCL, SENSOR2_RESET);
}

void loop() {
    uint16_t sensor1Data, sensor2Data;

    readSensor(sensor1, SENSOR1_RESET, &sensor1Data);
    readSensor(sensor2, SENSOR2_RESET, &sensor2Data);

    Serial.print("Sensor 1: ");
    Serial.print(sensor1Data);
    Serial.print(" mm, Sensor 2: ");
    Serial.print(sensor2Data);
    Serial.println(" mm");

    delay(100);
}


/*
#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 21      // I2C SDA
#define SCL_PIN 22      // I2C SCL
#define RESET_PIN 18    // XSHUT pin for resetting sensor
#define SENSOR_TIMEOUT 1000 // Timeout to detect sensor failure (ms)

VL53L0X sensor;
unsigned long lastValidRead = 0; // Stores last successful read time

// Function to initialize the sensor
void initSensor() {
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, HIGH);  // Keep sensor active

    Wire.begin(SDA_PIN, SCL_PIN); // Start I2C communication

    if (!sensor.init()) {
        Serial.println("VL53L0X not detected! Resetting...");
        resetSensor();
    }
    sensor.setTimeout(500);
    sensor.startContinuous();
}

// Function to reset sensor using XSHUT
void resetSensor() {
    Serial.println("Resetting VL53L0X...");
    sensor.stopContinuous();
    Wire.end(); // Close I2C
    digitalWrite(RESET_PIN, LOW);  // Power down sensor
    delay(500);                    // Wait before re-enabling
    digitalWrite(RESET_PIN, HIGH); // Power up sensor
    delay(500);                    // Allow time to start up
    Wire.begin(SDA_PIN, SCL_PIN);  // Restart I2C
    if (!sensor.init()) {
        Serial.println("Sensor failed after reset! Restarting ESP...");
        ESP.restart();  // Restart ESP if sensor fails
    }
    sensor.setTimeout(500);
    sensor.startContinuous();
}

// Function to read sensor and handle errors
uint16_t getSensorData() {
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    
    if (sensor.timeoutOccurred() || distance > 1000) {
        Serial.println("Sensor error detected! Attempting reset...");
        resetSensor();
    } else {
        lastValidRead = millis(); // Update last successful read time
    }
    
    return distance;
}

void setup() {
    Serial.begin(115200);
    initSensor();
}

void loop() {
    uint16_t distance = getSensorData();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");

    delay(100); // Read every 100ms
}
*/
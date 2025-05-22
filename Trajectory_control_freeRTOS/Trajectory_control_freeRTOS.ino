#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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
#define MOVING_AVG_SIZE 10
#define PROPORTIONAL_GAIN 1

// VL53L0X objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Measurement data
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

int avg1 = 0, avg2 = 0;
int lastSensor1 = 0, lastSensor2 = 0;
int repeatCount = 0;
const int maxRepeats = 10;
SemaphoreHandle_t xMutex;

// Target positions
int targetPositions1[] = {168, 174, 178, 178, 176, 171, 164, 156, 147, 139};
int targetPositions2[] = {217, 209, 202, 197, 194, 195, 198, 204, 212, 220};
int targetIndex1 = 0, targetIndex2 = 0;

void setID() {
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to boot first VL53L0X. Retrying...");
        for (int i = 0; i < 3; i++) {
            delay(1000);
            if (lox1.begin(LOX1_ADDRESS)) {
                Serial.println("First VL53L0X initialized successfully!");
                return;
            }
        }
        Serial.println("Failed to initialize the first VL53L0X after retries.");
        return;
    }
    delay(10);

    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println("Failed to boot second VL53L0X. Retrying...");
        for (int i = 0; i < 3; i++) {
            delay(1000);
            if (lox2.begin(LOX2_ADDRESS)) {
                Serial.println("Second VL53L0X initialized successfully!");
                return;
            }
        }
        Serial.println("Failed to initialize the second VL53L0X after retries.");
        return;
    }
}

void readTOFTask(void *parameter) {
    while (1) {
        lox1.rangingTest(&measure1, false);
        lox2.rangingTest(&measure2, false);
        xSemaphoreTake(xMutex, portMAX_DELAY);

        avg1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : avg1;
        avg2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : avg2;

        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
       
void printTOFTask(void *parameter) {
    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        Serial.print("Sensor1: ");
        Serial.print(avg1);
        Serial.print("\t Sensor2: ");
        Serial.println(avg2);

        if (avg1 == lastSensor1 && avg2 == lastSensor2 && avg1 != 0 && avg2 != 0) {
            repeatCount++;
        } else {
            repeatCount = 0;
        }

        lastSensor1 = avg1;
        lastSensor2 = avg2;

        if (repeatCount >= maxRepeats) {
            Serial.println("Repeated readings detected! Reinitializing sensors...");
            setID();
            repeatCount = 0;
        }
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void motorControlTask(void *parameter) {
    int ActuatorSpeed = 150;
    bool motor1Reached = false, motor2Reached = false;

    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        int error1 = targetPositions1[targetIndex1] - avg1;
        int error2 = targetPositions2[targetIndex2] - avg2;
        int controlSignal1 = PROPORTIONAL_GAIN * error1;
        int controlSignal2 = PROPORTIONAL_GAIN * error2;
        xSemaphoreGive(xMutex);

        if (abs(error1) > 2) {
            digitalWrite(IN1, controlSignal1 > 0 ? HIGH : LOW);
            digitalWrite(IN2, controlSignal1 > 0 ? LOW : HIGH);
            analogWrite(ENA, ActuatorSpeed);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            motor1Reached = true;
        }

        if (abs(error2) > 2) {
            digitalWrite(IN3, controlSignal2 > 0 ? HIGH : LOW);
            digitalWrite(IN4, controlSignal2 > 0 ? LOW : HIGH);
            analogWrite(ENB, ActuatorSpeed);
        } else {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            motor2Reached = true;
        }

        if (motor1Reached && motor2Reached) {
            Serial.print(F("Both actuators reached their targets at index: "));
            Serial.println(targetIndex1);
            targetIndex1 = (targetIndex1 + 1) % 10;
            targetIndex2 = (targetIndex2 + 1) % 10;
            motor1Reached = false;
            motor2Reached = false;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void setup() {
    Serial.begin(9600);
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
    setID();

    xMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(readTOFTask, "Read TOF", 3072, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(printTOFTask, "Print TOF", 3072, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(motorControlTask, "Motor Control", 3072, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}

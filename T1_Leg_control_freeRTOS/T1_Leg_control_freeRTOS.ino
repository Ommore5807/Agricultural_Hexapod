#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <esp_now.h>
#include <WiFi.h>

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

// Proportional Gain
#define PROPORTIONAL_GAIN 1

// MAC Address of R1 (Change this)
uint8_t R1_MAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};

// VL53L0X objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Measurement data
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

int avg1 = 0, avg2 = 0;
SemaphoreHandle_t xMutex;

// Motor activation flag
bool startMotor = false;

// Target positions
int targetPositions1[] = {168, 174, 178, 178, 176, 171, 164, 156, 147, 139};
int targetPositions2[] = {217, 209, 202, 197, 194, 195, 198, 204, 212, 220};
int targetIndex1 = 0, targetIndex2 = 0;

// Data structure for ESP-NOW communication
typedef struct {
    int tof1;
    int tof2;
} DataPacket;

typedef struct {
    char command[10];  // Command from R1 (e.g., "START", "STOP")
} CommandPacket;

DataPacket tofData;

// Function to initialize VL53L0X sensors
void setID() {
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);

    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to initialize VL53L0X-1");
        return;
    }
    delay(10);

    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println("Failed to initialize VL53L0X-2");
        return;
    }
}

// Task to read TOF sensors
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

// Task to print TOF sensor values
void printTOFTask(void *parameter) {
    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        Serial.print("TOF1: ");
        Serial.print(avg1);
        Serial.print("\t TOF2: ");
        Serial.println(avg2);
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Task to send TOF values to R1
void sendTOFTask(void *parameter) {
    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        tofData.tof1 = avg1;
        tofData.tof2 = avg2;
        xSemaphoreGive(xMutex);

        esp_now_send(R1_MAC, (uint8_t *)&tofData, sizeof(tofData));
        Serial.println("TOF Data Sent to R1");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task to control motors (Only when startMotor = true)
void motorControlTask(void *parameter) {
    int ActuatorSpeed = 150;
    bool motor1Reached = false, motor2Reached = false;

    while (1) {
        if (startMotor) {
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
        } else {
            // Stop motors when startMotor = false
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Callback function for ESP-NOW data reception
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    CommandPacket receivedCommand;
    memcpy(&receivedCommand, incomingData, sizeof(receivedCommand));

    Serial.print("Command received: ");
    Serial.println(receivedCommand.command);

    if (strcmp(receivedCommand.command, "START") == 0) {
        startMotor = true;
    } else if (strcmp(receivedCommand.command, "STOP") == 0) {
        startMotor = false;
    }
}

// Setup function
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

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, R1_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    esp_now_register_recv_cb(onDataRecv);  // Register receive callback

    xMutex = xSemaphoreCreateMutex();
    xTaskCreate(readTOFTask, "Read TOF", 3072, NULL, 2, NULL);
    xTaskCreate(printTOFTask, "Print TOF", 3072, NULL, 2, NULL);
    xTaskCreate(sendTOFTask, "Send TOF", 3072, NULL, 2, NULL);
    xTaskCreate(motorControlTask, "Motor Control", 3072, NULL, 1, NULL);
}

void loop() {}


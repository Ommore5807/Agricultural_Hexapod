/*
#include <esp_now.h>
#include <WiFi.h>

uint8_t receiverMAC[] = {0x4c, 0x11, 0xae, 0x66, 0x67, 0x14}; // Replace with your receiver ESP32 MAC address

typedef struct struct_message {
    float tof1;
    float tof2;
} struct_message;

struct_message sensorData;

void sendData() {
    sensorData.tof1 = 100.0;  // Replace with actual ToF sensor reading
    sensorData.tof2 = 200.0;  // Replace with actual ToF sensor reading
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&sensorData, sizeof(sensorData));

    if (result == ESP_OK) {
        Serial.println("Sent successfully");
    } else {
        Serial.println("Error sending");
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
    }
    
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("ESP-NOW ready");
}

void loop() {
    sendData();
    delay(500);  // Adjust as needed
}
*/
/*
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Replace with your receiver ESP32 MAC address
uint8_t receiverMAC[] = {0x4c, 0x11, 0xae, 0x66, 0x67, 0x14};

// Define ToF Sensor Addresses and Shutdown Pins
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define SHT_LOX1 4
#define SHT_LOX2 5

// VL53L0X Sensor Objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

typedef struct struct_message {
    float tof1;
    float tof2;
} struct_message;

struct_message sensorData;

// Function to set unique I2C addresses for two ToF sensors
void setID() {
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    // Reset both sensors
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    // Activate first sensor, deactivate second
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    // Initialize first sensor
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to initialize VL53L0X sensor 1");
        while (1);
    }

    // Activate second sensor
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // Initialize second sensor
    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println("Failed to initialize VL53L0X sensor 2");
        while (1);
    }

    Serial.println("Both ToF sensors initialized");
}

// Function to read ToF sensor data
void readToFSensors() {
    VL53L0X_RangingMeasurementData_t measure1, measure2;

    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);

    sensorData.tof1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
    sensorData.tof2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;

    Serial.print("ToF1: ");
    Serial.print(sensorData.tof1);
    Serial.print(" mm, ToF2: ");
    Serial.println(sensorData.tof2);
}

// Function to send data via ESP-NOW
void sendData() {
    readToFSensors();  // Read sensor data before sending

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&sensorData, sizeof(sensorData));

    if (result == ESP_OK) {
        Serial.println("Sent successfully");
    } else {
        Serial.println("Error sending");
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
    }

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Initialize ToF sensors
    setID();
    
    Serial.println("ESP-NOW ready");
}

void loop() {
    sendData();
    delay(500);  // Adjust as needed
}
*/
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// Define ToF Sensor Addresses and Shutdown Pins
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define SHT_LOX1 4
#define SHT_LOX2 5

// Motor control pins
#define ENA 33
#define IN1 26
#define IN2 27

// VL53L0X Sensor Objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Define data structures
typedef struct struct_message {
    float tof1;
    float tof2;
} struct_message;

typedef struct motor_command {
    int command;  // 1 = Forward, 2 = Backward
} motor_command;

struct_message sensorData;
motor_command motorCmd;

uint8_t receiverMAC[] = {0x4c, 0x11, 0xae, 0x66, 0x67, 0x14};  // Replace with actual receiver MAC

// Function to set unique I2C addresses for two ToF sensors
void setID() {
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println("Failed to initialize VL53L0X sensor 1");
        while (1);
    }

    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println("Failed to initialize VL53L0X sensor 2");
        while (1);
    }

    Serial.println("Both ToF sensors initialized");
}

// Function to read ToF sensor data
void readToFSensors() {
    VL53L0X_RangingMeasurementData_t measure1, measure2;

    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);

    sensorData.tof1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
    sensorData.tof2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;

    Serial.print("ToF1: ");
    Serial.print(sensorData.tof1);
    Serial.print(" mm, ToF2: ");
    Serial.println(sensorData.tof2);
}

// Function to send sensor data to receiver
void sendData() {
    readToFSensors();
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&sensorData, sizeof(sensorData));

    if (result == ESP_OK) {
        Serial.println("Sensor data sent successfully");
    } else {
        Serial.println("Error sending sensor data");
    }
}

// Callback function to receive motor commands
void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&motorCmd, incomingData, sizeof(motorCmd));

    if (motorCmd.command == 1) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 255);
        Serial.println("Motor Moving Forward");
    } else if (motorCmd.command == 2) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 255);
        Serial.println("Motor Moving Backward");
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    setID();
    Serial.println("ESP-NOW ready");
}

void loop() {
    sendData();
    delay(500);
}



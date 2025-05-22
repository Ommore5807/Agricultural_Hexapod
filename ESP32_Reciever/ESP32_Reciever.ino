#include <esp_now.h>
#include <WiFi.h>

// Define motor control pins
#define ENA 33
#define IN1 26
#define IN2 27

typedef struct struct_message {
    float tof1;
    float tof2;
} struct_message;

typedef struct motor_command {
    int command;  // 1 = Forward, 2 = Backward
} motor_command;

struct_message sensorData;
motor_command motorCmd;

uint8_t transmitterMAC[] = {0x2c, 0xbc, 0xbb, 0x0c, 0x5c, 0x0c};  // Replace with actual transmitter MAC

int mode = 0;  // 0 = Stop, 1 = Forward, 2 = Backward

void moveMotor(int command) {
    if (command == 1) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 255);  // Full speed forward
        Serial.println("Motor Moving Forward");
    } else if (command == 2) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 255);  // Full speed backward
        Serial.println("Motor Moving Backward");
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);  // Stop motor
        Serial.println("Motor Stopped");
    }
}

// ESP-NOW callback to receive sensor data
void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&sensorData, incomingData, sizeof(sensorData));
    Serial.print("Received ToF1: ");
    Serial.print(sensorData.tof1);
    Serial.print(" mm, ToF2: ");
    Serial.println(sensorData.tof2);
}

// Send motor control command to transmitter
void sendMotorCommand(int command) {
    motorCmd.command = command;
    esp_err_t result = esp_now_send(transmitterMAC, (uint8_t*)&motorCmd, sizeof(motorCmd));

    if (result == ESP_OK) {
        Serial.print("Motor command sent: ");
        Serial.println(command);
    } else {
        Serial.println("Error sending motor command");
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

    // Register the transmitter as a peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, transmitterMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("ESP-NOW ready to receive and send motor commands");
    Serial.println("Enter mode: 1 = Forward, 2 = Backward, 0 = Stop");
}

void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        if (input >= '0' && input <= '2') {
            mode = input - '0';
            Serial.print("Selected mode: ");
            Serial.println(mode);
        }
    }

    switch (mode) {
        case 1:
            sendMotorCommand(1);
            moveMotor(1);
            break;
        case 2:
            sendMotorCommand(2);
            moveMotor(2);
            break;
        default:
            sendMotorCommand(0);
            moveMotor(0);
            break;
    }

    delay(500);  // Adjust delay as needed
}

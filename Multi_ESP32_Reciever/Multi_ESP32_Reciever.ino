#include <esp_now.h>
#include <WiFi.h>



// Structure to receive sensor data
typedef struct struct_message {
    float tof1;
    float tof2;
} struct_message;

// Structure to send motor command
typedef struct motor_command {
    int command;  // 0 = Stop, 1 = Forward, 2 = Backward
} motor_command;

// Sensor data from both ESP32 senders
struct_message sensorData1;
struct_message sensorData2;
motor_command motorCmd;

uint8_t sender1MAC[] = {0x2c, 0xbc, 0xbb, 0x0c, 0x5c, 0x0c};  // MAC of ESP 1
uint8_t sender2MAC[] = {0x4c, 0x11, 0xae, 0x66, 0x67, 0x14};  // MAC of ESP 2
uint8_t receiverMAC[] = {0x4c, 0x11, 0xae, 0x66, 0x67, 0x15}; // MAC of the receiver (this ESP32)

int mode = 0;  // 0 = Stop, 1 = Forward, 2 = Backward


// Send motor control command to both senders
void sendMotorCommand(int command) {
    motorCmd.command = command;
    
    esp_err_t result1 = esp_now_send(sender1MAC, (uint8_t*)&motorCmd, sizeof(motorCmd));
    esp_err_t result2 = esp_now_send(sender2MAC, (uint8_t*)&motorCmd, sizeof(motorCmd));

    if (result1 == ESP_OK && result2 == ESP_OK) {
        Serial.print("Motor command sent: ");
        Serial.println(command);
    } else {
        Serial.println("Error sending motor command");
    }
}

// ESP-NOW callback to receive sensor data from both ESPs
void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (memcmp(info->src_addr, sender1MAC, 6) == 0) {
        memcpy(&sensorData1, incomingData, sizeof(sensorData1));
        Serial.print("Sender 1 - ToF1: ");
        Serial.print(sensorData1.tof1);
        Serial.print(" mm, ToF2: ");
        Serial.println(sensorData1.tof2);
    } 
    else if (memcmp(info->src_addr, sender2MAC, 6) == 0) {
        memcpy(&sensorData2, incomingData, sizeof(sensorData2));
        Serial.print("Sender 2 - ToF1: ");
        Serial.print(sensorData2.tof1);
        Serial.print(" mm, ToF2: ");
        Serial.println(sensorData2.tof2);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

   

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);

    // Register sender 1 as a peer
    esp_now_peer_info_t peerInfo1;
    memcpy(peerInfo1.peer_addr, sender1MAC, 6);
    peerInfo1.channel = 0;
    peerInfo1.encrypt = false;

    if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
        Serial.println("Failed to add sender 1");
    }

    // Register sender 2 as a peer
    esp_now_peer_info_t peerInfo2;
    memcpy(peerInfo2.peer_addr, sender2MAC, 6);
    peerInfo2.channel = 0;
    peerInfo2.encrypt = false;

    if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
        Serial.println("Failed to add sender 2");
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
}

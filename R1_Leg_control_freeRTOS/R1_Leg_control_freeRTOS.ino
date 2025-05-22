#include <esp_now.h>
#include <WiFi.h>

// MAC Addresses (Update with actual MACs)
uint8_t T1_MAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
uint8_t T2_MAC[] = {0xYY, 0xYY, 0xYY, 0xYY, 0xYY, 0xYY};

// Structure for receiving TOF data from T1
typedef struct {
    int tof1;
    int tof2;
} DataPacket;

typedef struct {
    char command[10];  // Command to send ("START" / "STOP")
} CommandPacket;

DataPacket tofData;
bool taskCompleted = false;

// ESP-NOW Send Function
void sendCommandToT1(const char *cmd) {
    CommandPacket commandPacket;
    strcpy(commandPacket.command, cmd);
    esp_now_send(T1_MAC, (uint8_t *)&commandPacket, sizeof(commandPacket));
    Serial.print("Sent command to T1: ");
    Serial.println(cmd);
}

// ESP-NOW Send Function to T2
void sendDataToT2() {
    esp_now_send(T2_MAC, (uint8_t *)&tofData, sizeof(tofData));
    Serial.println("Sent TOF data to T2");
}

// ESP-NOW Receive Callback (Handles TOF data from T1)
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&tofData, incomingData, sizeof(tofData));
    
    Serial.print("Received TOF1: ");
    Serial.print(tofData.tof1);
    Serial.print("\tTOF2: ");
    Serial.println(tofData.tof2);

    // Task completion check
    if (tofData.tof1 >= 200 && tofData.tof2 >= 200) { // Adjust threshold if needed
        Serial.println("Task completed, stopping T1 and switching to T2");
        sendCommandToT1("STOP");
        taskCompleted = true;
    }
}

// ESP-NOW Send Status Callback
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\nSend Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Setup function
void setup() {
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo;

    // Add T1 as Peer
    memcpy(peerInfo.peer_addr, T1_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    // Add T2 as Peer
    memcpy(peerInfo.peer_addr, T2_MAC, 6);
    esp_now_add_peer(&peerInfo);

    // Start T1 task
    sendCommandToT1("START");
}

void loop() {
    if (taskCompleted) {
        sendDataToT2();
        delay(500); // Adjust timing as needed
    }
}


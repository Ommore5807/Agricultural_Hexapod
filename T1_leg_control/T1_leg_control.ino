#include <WiFi.h>
#include <esp_now.h>

#define NUM_RECEIVERS 6

// Replace these MAC addresses with actual values
uint8_t R_MACS[NUM_RECEIVERS][6] = {
    {0xc0, 0x49, 0xef, 0x69, 0x90, 0x50}, // RF
    {0x4C, 0x11, 0xae, 0x66, 0x67, 0x14}, // LM
    {0x78, 0x21, 0x84, 0x7a, 0x01, 0x30}, // RR
    {0x08, 0xa6, 0xf7, 0x5a, 0x8d, 0xdc}, // LF
    {0x2C, 0xBC, 0xBB, 0x0D, 0xF2, 0xb0}, // RM
    {0x08, 0xa6, 0xf, 0x5a, 0xc5, 0xdc}  // LR
};

int currentReceiverIndex = -1; // Tracks which receiver we're currently communicating with
char commandToSend = '0';
bool waitingForAck = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Data sent successfully!" : "Failed to send data!");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    String incoming = String((char*)incomingData);

    Serial.print("Received: ");
    Serial.println(incoming);

    if (waitingForAck && incoming == "done") {
        waitingForAck = false;
        currentReceiverIndex++;

        if (currentReceiverIndex < NUM_RECEIVERS) {
            sendToReceiver(currentReceiverIndex, commandToSend);
            waitingForAck = true;
        } else {
            Serial.println("Sequence complete. All receivers processed.");
            currentReceiverIndex = -1; // Reset
        }
    }
}

void addPeer(uint8_t *mac) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (!esp_now_is_peer_exist(mac)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add peer!");
        }
    }
}

void sendToReceiver(int index, char command) {
    addPeer(R_MACS[index]);
    Serial.printf("Sending command %c to R%d...\n", command, index + 1);
    esp_now_send(R_MACS[index], (uint8_t *)&command, sizeof(command));
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  
    if (Serial.available()) {
        char input = Serial.read();

        switch (input) {
    case '1':
        if (!waitingForAck && currentReceiverIndex == -1) {
            commandToSend = '1';
            currentReceiverIndex = 0;
            sendToReceiver(currentReceiverIndex, commandToSend);
            waitingForAck = true;
        } else {
            Serial.println("Still waiting for current sequence to complete.");
        }
        break;

    case '2':
        if (!waitingForAck && currentReceiverIndex == -1) {
            commandToSend = '2';
            currentReceiverIndex = 0;
            sendToReceiver(currentReceiverIndex, commandToSend);
            waitingForAck = true;
        } else {
            Serial.println("Still waiting for current sequence to complete.");
        }
        break;

    default:
        Serial.println("Invalid input. Press 1 or 2.");
        break;
      }

    }
}

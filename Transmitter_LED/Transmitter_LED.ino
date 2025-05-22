
#include <esp_now.h>
#include <WiFi.h>

#define LED_BUILTIN 2
uint8_t receiverMAC[] = {0x08, 0xa6, 0xf7, 0xa8, 0x76, 0x34}; // MAC of Receiver (R1)

typedef struct {
    char message[20];
} Message;

Message receivedData, sendData;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received: ");
    Serial.println(receivedData.message);

    if (strcmp(receivedData.message, "START") == 0) {
        // Blink LED 6 times
        for (int i = 0; i < 6; i++) {
            digitalWrite(LED_BUILTIN, HIGH);  
            delay(1000);                      
            digitalWrite(LED_BUILTIN, LOW);   
            delay(1000);                      
        }

        // Send "DONE" response back to Receiver (R1)
        strcpy(sendData.message, "DONE");
        esp_now_send(receiverMAC, (uint8_t *)&sendData, sizeof(sendData));
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    pinMode(LED_BUILTIN, OUTPUT);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
}

void loop() {
    // Do nothing, wait for messages
}

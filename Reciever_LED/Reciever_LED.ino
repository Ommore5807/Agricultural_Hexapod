#include <esp_now.h>
#include <WiFi.h>

uint8_t transmitterMACs[6][6] = {
    {0x4c, 0x11, 0xae, 0x66, 0x67, 0x14},  // MAC address of T1(ESP NO.1 )
    //{0x88, 0x13, 0xbf, 0x6f, 0x82, 0x58},  // MAC address of T2(ESP NO.2 )
    //{0x88, 0x13, 0xbf, 0x6f, 0x8d, 0xd0},  // MAC address of T3(ESP NO.3 )
    {0x2c, 0xbc, 0xbb, 0x0d, 0xf2, 0xb0},  // MAC address of T4(ESP NO.4 )
    //{0xac, 0x15, 0x18, 0xeb, 0x71, 0x0c},  // MAC address of T5(ESP NO.5 )
    {0x08, 0xa6, 0xf7, 0x5a, 0xc5, 0xdc}   // MAC address of T6(ESP NO.7 )
};

int currentTransmitter = 0;
bool receivedResponse = false;

typedef struct {
    char message[20];
} Message;

Message sendData, receivedData;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received from Transmitter: ");
    Serial.println(receivedData.message);
    receivedResponse = true;
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    for (int i = 0; i < 6; i++) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, transmitterMACs[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add peer");
        }
    }
}

void loop() {
    if (currentTransmitter < 6) {
        strcpy(sendData.message, "START");
        receivedResponse = false;

        Serial.print("Sending to T");
        Serial.println(currentTransmitter + 1);

        esp_err_t result = esp_now_send(transmitterMACs[currentTransmitter], (uint8_t *)&sendData, sizeof(sendData));
        if (result == ESP_OK) {
            Serial.println("Message sent");
        } else {
            Serial.println("Message send failed");
        }

        unsigned long startTime = millis();
        while (!receivedResponse && millis() - startTime < 5000) {
            // Wait for response with timeout
        }

        if (receivedResponse) {
            Serial.print("T");
            Serial.print(currentTransmitter + 1);
            Serial.println(" completed!");
            currentTransmitter++;
        } else {
            Serial.println("No response, retrying...");
        }

        delay(1000);
    }
}


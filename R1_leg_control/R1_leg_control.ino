#include <WiFi.h>
#include <esp_now.h>

// ---------------- MOTOR CONTROL CODE (UNCHANGED) ----------------

#define ENA 33   // PWM pin for Motor 1
#define IN1 26
#define IN2 27
#define ENB 12   // PWM pin for Motor 2
#define IN3 14
#define IN4 32 

#define PWM_Value 255
#define STEP_DURATION 10
#define SPEED_MM_SEC 10

int motor1_trajectory[] = {128, 140, 146, 145, 139, 128, 113, 96, 79, 62};
int motor2_trajectory[] = {177, 160, 145, 135, 130, 133, 142, 156, 172, 188};
int num_points = sizeof(motor1_trajectory) / sizeof(motor1_trajectory[0]);

enum State { WAIT_FOR_START, RUNNING, DONE };
State currentState = WAIT_FOR_START;

uint8_t T1_MAC[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};  // <-- Replace with actual MAC of T1

void moveMotors(int delta1, int delta2) {
  unsigned long time1 = abs(delta1) / (float)SPEED_MM_SEC * 1000;
  unsigned long time2 = abs(delta2) / (float)SPEED_MM_SEC * 1000;
  unsigned long maxTime = max(time1, time2);

  if (delta1 > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (delta1 < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }

  if (delta2 > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (delta2 < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }

  if (delta1 != 0) analogWrite(ENA, PWM_Value);
  if (delta2 != 0) analogWrite(ENB, PWM_Value);

  unsigned long startTime = millis();
  bool motor1Running = (delta1 != 0);
  bool motor2Running = (delta2 != 0);

  while (millis() - startTime < maxTime) {
    unsigned long elapsed = millis() - startTime;
    if (motor1Running && elapsed >= time1) {
      analogWrite(ENA, 0);
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
      motor1Running = false;
    } 
    if (motor2Running && elapsed >= time2) {
      analogWrite(ENB, 0);
      digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
      motor2Running = false;
    }
    delay(1);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void retractMotors() {
  analogWrite(ENA, 150);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENB, 150);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  delay(20000);  // Simulate retract time
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}



// -------------------- ESP-NOW + MAIN LOGIC ---------------------

String incomingMsg = "";
bool actionComplete = false;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  char msg = (char)data[0];

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);

  Serial.print("Received message: "); Serial.println(msg);
  Serial.print("From MAC: "); Serial.println(macStr);

  switch (msg) {
    case '1':
      startTrajectory();
      
      break;
    case '2':
      retractMotors();
      sendDone();
      break;
  }
}

void sendDone() {
  const char *doneMsg = "done";
  esp_err_t result = esp_now_send(T1_MAC, (uint8_t *)doneMsg, strlen(doneMsg));
  if (result == ESP_OK) {
    Serial.println("Sent: done to T1");
  } else {
    Serial.print("Send failed, error: ");
    Serial.println(result);
  }
}

void startTrajectory() {
  static int prev_motor1 = 0;
  static int prev_motor2 = 0;

  Serial.println("Starting trajectory...");
  for (int i = 0; i < num_points; i++) {
    int delta1 = motor1_trajectory[i] - prev_motor1;
    int delta2 = motor2_trajectory[i] - prev_motor2;

    Serial.printf("Moving to point %d: M1=%d mm, M2=%d mm\n", i + 1, delta1, delta2);
    moveMotors(delta1, delta2);
    prev_motor1 = motor1_trajectory[i];
    prev_motor2 = motor2_trajectory[i];
    delay(300);
  }

  Serial.println("Trajectory done.");
  sendDone();
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Register T1 as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, T1_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer (T1)");
  } else {
    Serial.println("Peer (T1) added");
  }
}

void loop() {
  // Nothing needed here
}

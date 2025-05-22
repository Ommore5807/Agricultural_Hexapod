
#include <Arduino.h>
#include <Wire.h>



#define ENA 33   // PWM pin for Motor 1
#define IN1 26
#define IN2 27
#define ENB 12   // PWM pin for Motor 2
#define IN3 14
#define IN4 32 

#define PWM_Value 255
#define SPEED_MM_SEC 10

// All trajectory sets
int motor1_trajectories[8][10] = {
  {128, 140, 146, 145, 139, 128, 113, 96, 79, 62},    // Array 1
  {128, 143, 157, 170, 180, 186, 188, 185, 176, 163}, // Array 3
  {175, 181, 183, 180, 172, 160, 146, 132,120,110}, // Array 5
  {175, 184, 195, 206, 215, 221, 223, 222, 218, 211}  // Array 7
};

int motor2_trajectories[8][10] = {
  {177, 160, 145, 135, 130, 133, 142, 156, 172, 188}, // Array 2
  {177, 160, 145, 135, 130, 133, 142, 156, 172, 188}, // Array 4
  {102, 84, 67, 55, 51, 54, 65, 81, 100, 119},        // Array 6
  {102, 84, 67, 55, 51, 54, 65, 81, 100, 119}  // Array 8
};

int num_points = 10;
int currentSet = 0;

enum State { WAIT_FOR_START, RUNNING };
State currentState = WAIT_FOR_START;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.begin(115200);
  Serial.println("Send '1', '4', '5', or '6' to start specific trajectory pair.\nSend '2' to retract.\nSend '3' to extend.");
}

void moveMotors(int delta1, int delta2) {
  unsigned long time1 = abs(delta1) / (float)SPEED_MM_SEC * 1000;
  unsigned long time2 = abs(delta2) / (float)SPEED_MM_SEC * 1000;
  unsigned long maxTime = max(time1, time2);

  if (delta1 > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (delta1 < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }

  if (delta2 > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (delta2 < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

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

  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void retractMotors() {
  Serial.println("Retracting...");
  analogWrite(ENA, 150); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENB, 150); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void extendMotors(int current1, int current2) {
  Serial.println("Extending...");
  int* traj1 = motor1_trajectories[currentSet];
  int* traj2 = motor2_trajectories[currentSet];
  moveMotors(traj1[num_points - 1] - current1, traj2[num_points - 1] - current2);
}

void loop() {
  static int prev_motor1 = 0;
  static int prev_motor2 = 0;
  static int i = 0;

  switch (currentState) {
    case WAIT_FOR_START:
      if (Serial.available()) {
        char command = Serial.read();
        if (command == '1') { currentSet = 0; }       // Use arrays 1 & 2
        else if (command == '4') { currentSet = 1; }  // Use arrays 3 & 4
        else if (command == '5') { currentSet = 2; }  // Use arrays 5 & 6
        else if (command == '6') { currentSet = 3; }  // Use arrays 7 & 8

        if (command == '1' || command == '4' || command == '5' || command == '6') {
          Serial.println("Starting trajectory...");
          prev_motor1 = 0; prev_motor2 = 0; i = 0;
          currentState = RUNNING;
        } else if (command == '2') {
          retractMotors(); prev_motor1 = 0; prev_motor2 = 0;
        } else if (command == '3') {
          extendMotors(prev_motor1, prev_motor2);
          prev_motor1 = motor1_trajectories[currentSet][num_points - 1];
          prev_motor2 = motor2_trajectories[currentSet][num_points - 1];
        }
      }
      break;

    case RUNNING:
      if (i < num_points) {
        int* traj1 = motor1_trajectories[currentSet];
        int* traj2 = motor2_trajectories[currentSet];

        int delta1 = traj1[i] - prev_motor1;
        int delta2 = traj2[i] - prev_motor2;

        Serial.print("Point "); Serial.print(i + 1);
        Serial.print(": M1 -> "); Serial.print(delta1);
        Serial.print(" mm, M2 -> "); Serial.println(delta2);

        moveMotors(delta1, delta2);
        prev_motor1 = traj1[i]; prev_motor2 = traj2[i];
        i++;
        delay(500);
      } else {
        Serial.println("Trajectory complete. Send '1', '4', '5', or '6' to repeat.");
        currentState = WAIT_FOR_START;
      }
      break;
  }
}

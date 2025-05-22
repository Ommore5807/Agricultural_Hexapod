#define ENA 33   // PWM pin for Motor 1
#define IN1 26
#define IN2 27
#define ENB 12   // PWM pin for Motor 2
#define IN3 14
#define IN4 32 

#define PWM_Value 255        // Set PWM speed (0-255, 128 is ~50% speed)
#define STEP_DURATION 10     // Base step delay (not used for stepping now)
#define SPEED_MM_SEC 10      // Speed at given PWM value (mm/sec)

// Trajectory points (in mm)
int motor1_trajectory[] = {128, 140, 146, 145, 139, 128, 113, 96, 79, 62};
int motor2_trajectory[] = {177, 160, 145, 135, 130, 133, 142, 156, 172, 188};
int num_points = sizeof(motor1_trajectory) / sizeof(motor1_trajectory[0]);

enum State { WAIT_FOR_START, RUNNING, DONE };
State currentState = WAIT_FOR_START;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(115200);
  Serial.println("Send '1' to start motor control.\nSend '2' to retract.\nSend '3' to extend.");
}

void moveMotors(int delta1, int delta2) {
  unsigned long time1 = abs(delta1) / (float)SPEED_MM_SEC * 1000;
  unsigned long time2 = abs(delta2) / (float)SPEED_MM_SEC * 1000;
  unsigned long maxTime = max(time1, time2);

  Serial.print("Time1 (M1): ");
  Serial.print(time1);
  Serial.print(" ms, Time2 (M2): ");
  Serial.print(time2);
  Serial.println(" ms");

  if (delta1 > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (delta1 < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (delta2 > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (delta2 < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  if (delta1 != 0)
    analogWrite(ENA, PWM_Value);
  if (delta2 != 0)
    analogWrite(ENB, PWM_Value);

  unsigned long startTime = millis();
  bool motor1Running = (delta1 != 0);
  bool motor2Running = (delta2 != 0);

  while (millis() - startTime < maxTime) {
    unsigned long elapsed = millis() - startTime;

    if (motor1Running && elapsed >= time1) {
      analogWrite(ENA, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      motor1Running = false;
    } 
    if (motor2Running && elapsed >= time2) {
      analogWrite(ENB, 0);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      motor2Running = false;
    }

    delay(1); // short delay to avoid busy wait
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// 🚀 New helper function to retract both motors
void retractMotors() {
  Serial.println("Retracting both motors to 0 mm...");
  analogWrite(ENA, 150);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
  analogWrite(ENB, 150);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
}

// 🚀 New helper function to extend both motors to last trajectory point
void extendMotors(int current1, int current2) {
  Serial.println("Extending both motors to final point...");
  moveMotors(motor1_trajectory[num_points - 1] - current1, motor2_trajectory[num_points - 1] - current2);
}

void loop() {
  static int prev_motor1 = 0;
  static int prev_motor2 = 0;
  static int i = 0;

  switch (currentState) {
    case WAIT_FOR_START:
      if (Serial.available()) {
        char command = Serial.read();
        if (command == '1') {
          Serial.println("Starting motor movement...");
          prev_motor1 = 0;
          prev_motor2 = 0;
          i = 0;
          currentState = RUNNING;
        } else if (command == '2') {
          retractMotors();
          prev_motor1 = 0;
          prev_motor2 = 0;
        } else if (command == '3') {
          extendMotors(prev_motor1, prev_motor2);
          prev_motor1 = motor1_trajectory[num_points - 1];
          prev_motor2 = motor2_trajectory[num_points - 1];
        }
      }
      break;

    case RUNNING:
      if (i < num_points) {
        int delta1 = motor1_trajectory[i] - prev_motor1;
        int delta2 = motor2_trajectory[i] - prev_motor2;

        Serial.print("Moving to point ");
        Serial.print(i + 1);
        Serial.print(": M1 -> ");
        Serial.print(delta1);
        Serial.print(" mm, M2 -> ");
        Serial.print(delta2);
        Serial.println(" mm");

        moveMotors(delta1, delta2);

        prev_motor1 = motor1_trajectory[i];
        prev_motor2 = motor2_trajectory[i];
        i++;

        delay(500);  // Short pause between steps
      } else {
        Serial.println("Trajectory complete. Send '1' to repeat.");
        currentState = WAIT_FOR_START;
      }
      break;

    case DONE:
      break;
  }
}

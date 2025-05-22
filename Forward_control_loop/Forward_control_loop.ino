#define ENA 33   // PWM pin for Motor 1
#define IN1 26
#define IN2 27
#define ENB 12   // PWM pin for Motor 2
#define IN3 14
#define IN4 32 

#define PWM_Value 255        // Set PWM speed (0-255)
#define SPEED_MM_SEC 10      // Speed at given PWM value (mm/sec)

// Updated Trajectory points (in mm)
int motor1_trajectory[] = {
  128, 152, 165, 168, 163, 152, 136, 115, 92, 62
};

int motor2_trajectory[] = {
  177, 144, 112, 89, 77, 81, 97, 124, 157, 188 
};

int num_points = sizeof(motor1_trajectory) / sizeof(motor1_trajectory[0]);

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(115200);
}

void moveMotors(int delta1, int delta2) {
  unsigned long time1 = abs(delta1) / (float)SPEED_MM_SEC * 1000;
  unsigned long time2 = abs(delta2) / (float)SPEED_MM_SEC * 1000;
  unsigned long maxTime = max(time1, time2);

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

void loop() {
  static int prev_motor1 = 0;
  static int prev_motor2 = 0;
  static int i = 0;
  static bool trajectoryDone = false;

  if (!trajectoryDone) {
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

      delay(500); // short pause between moves
    } else {
      Serial.println("Trajectory complete.");
      trajectoryDone = true;
    }
  }
}

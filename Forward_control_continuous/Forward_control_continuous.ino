#define ENA 33   // PWM pin for Motor 1
#define IN1 26
#define IN2 27
#define ENB 12   // PWM pin for Motor 2
#define IN3 14
#define IN4 32

#define PWM_Value 255
#define SPEED_MM_SEC 10

// New trajectory arrays
int array1[] = {128};
//int array1[] = {128,152,165,168,163,152,136,115,92,62,62,71,80,88,96,103,110,116,122,128,128,152,165,168,163,152,136,115,92,62,62,71,80,88,96,103,110,116,122,128,128,152,165,168,163,152,136,115,92,62,62,71,80,88,96,103,110,116,122,128};
int array2[] = {177};
//int array2[] = {177,144,112,89,77,81,97,124,157,188,188,186,184,182,181,180,179,178,177,177,177,144,112,89,77,81,97,124,157,188,188,186,184,182,181,180,179,178,177,177,177,144,112,89,77,81,97,124,157,188,188,186,184,182,181,180,179,178,177,177};
int array3[] = {128,155,176,193,205,212,211,204,189,163,163,161,158,155,151,147,143,138,133,128,128,155,176,193,205,212,211,204,189,163,163,161,158,155,151,147,143,138,133,128,128,155,176,193,205,212,211,204,189,163,163,161,158,155,151,147,143,138,133,128};
int array4[] = {177,144,112,89,77,81,97,124,157,188,188,186,184,182,181,180,179,178,177,177,177,144,112,89,77,81,97,124,157,188,188,186,184,182,181,180,179,178,177,177,177,144,112,89,77,81,97,124,157,188,188,186,184,182,181,180,179,178,177,177};

// Active trajectory pointers
int* motor1_trajectory = array1;
int* motor2_trajectory = array2;
int num_points = sizeof(array1) / sizeof(array1[0]);

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
  Serial.println("Send '1' to start array1+2.\nSend '2' to retract.\nSend '3' to use array3+4.");
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
    analogWrite(ENB, (PWM_Value));

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

    delay(1);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void retractMotors() {
  Serial.println("Retracting both motors to 0 mm...");
  analogWrite(ENA, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, 150);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

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
          Serial.println("Starting with Array1 + Array2...");
          motor1_trajectory = array1;
          motor2_trajectory = array2;
          num_points = sizeof(array1) / sizeof(array1[0]);
          prev_motor1 = 0;
          prev_motor2 = 0;
          i = 0;
          currentState = RUNNING;
        } else if (command == '2') {
          retractMotors();
          prev_motor1 = 0;
          prev_motor2 = 0;
        } else if (command == '3') {
          Serial.println("Switching to Array3 + Array4 and extending...");
          motor1_trajectory = array3;
          motor2_trajectory = array4;
          num_points = sizeof(array3) / sizeof(array3[0]);
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

        delay(500);
      } else {
        Serial.println("Trajectory complete. Send '1' or '3' to run again.");
        currentState = WAIT_FOR_START;
      }
      break;

    case DONE:
      break;
  }
}


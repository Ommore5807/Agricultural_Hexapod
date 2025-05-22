#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 32

void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.println("Starting actuator direction test...");
}

void loop() {
  /*
    Serial.println("Moving Actuator 1 Forward");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(2000);  // Move forward for 2 seconds
  */
  
    Serial.println("Moving Actuator 1 Backward");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    delay(2000);  // Move backward for 2 seconds
 /*
    Serial.println("Stopping Actuator 1");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(1000);  // Pause before switching actuators
  
    Serial.println("Moving Actuator 2 Forward");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(2000);  // Move forward for 2 seconds
  */
    Serial.println("Moving Actuator 2 Backward");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2000);  // Move backward for 2 seconds
  /*
    Serial.println("Stopping Actuator 2");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(2000);  // Pause before repeating
  */
}


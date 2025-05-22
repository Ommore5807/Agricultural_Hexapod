#include <Arduino.h>

#define ENA 33
#define IN1 26
#define IN2 27
#define ENB 12
#define IN3 14
#define IN4 32

void moveActuatorsToInitialPosition(int ActuatorSpeed, int duration) {
  // Move actuators backward to initial position
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, ActuatorSpeed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, ActuatorSpeed);
  delay(duration);
  /*
  // Move actuators backward to initial position
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, ActuatorSpeed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, ActuatorSpeed);
  */
  delay(duration);


}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println(F("Moving actuators to initial position..."));
 
}

void loop() {
  moveActuatorsToInitialPosition(100, 10000); // Adjust duration as needed
  Serial.println(F("Actuators in initial position."));
}


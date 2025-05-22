#include <Arduino.h>

// Define pins for the first actuator driver
#define IN1 26  // Control pin 1 for actuator 1
#define IN2 25  // Control pin 2 for actuator 1
#define ENA 27  // PWM pin for speed control for actuator 1

// Define pins for the second actuator driver
#define IN3 32  // Control pin 1 for actuator 2
#define IN4 35  // Control pin 2 for actuator 2
#define ENB 33  // PWM pin for speed control for actuator 2

void setup() {
  // Set control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
 /*
  // Configure PWM channels for ENA and ENB
  ledcSetup(0, 5000, 8); // Channel 0, 5 kHz frequency, 8-bit resolution
  ledcAttachPin(ENA, 0); // Attach ENA to PWM channel 0

  ledcSetup(1, 5000, 8); // Channel 1, 5 kHz frequency, 8-bit resolution
  ledcAttachPin(ENB, 1); // Attach ENB to PWM channel 1
 */
  Serial.begin(9600);
  Serial.println("Starting dual linear actuator control...");
}

void loop() {
  // Extend both actuators
  Serial.println("Extending actuators...");
  digitalWrite(IN1, HIGH); // Set actuator 1 to move forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Set actuator 2 to move forward
  digitalWrite(IN4, LOW);
  /*
  ledcWrite(0, 255);       // Full speed for actuator 1
  ledcWrite(1, 255);       // Full speed for actuator 2
  delay(5000);             // Wait for 5 seconds
  */
  // Retract both actuators
  Serial.println("Retracting actuators...");
  digitalWrite(IN1, LOW);  // Set actuator 1 to move backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Set actuator 2 to move backward
  digitalWrite(IN4, HIGH);
   /*
  ledcWrite(0, 255);       // Full speed for actuator 1
  ledcWrite(1, 255);       // Full speed for actuator 2
  
 */
 delay(5000);             // Wait for 5 seconds
  // Stop both actuators
  Serial.println("Stopping actuators...");
  digitalWrite(IN1, LOW);  // Stop actuator 1
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Stop actuator 2
  digitalWrite(IN4, LOW);
  /*
  ledcWrite(0, 0);         // Stop PWM for actuator 1
  ledcWrite(1, 0);         // Stop PWM for actuator 2
  */ 
  delay(5000);             // Wait for 5 seconds before repeating
}

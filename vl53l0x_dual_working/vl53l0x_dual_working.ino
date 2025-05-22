#include "Adafruit_VL53L0X.h"
#include <Wire.h>

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 4
#define SHT_LOX2 5

#define ENA 33
#define IN1 26
#define IN2 27
#define ENB 12
#define IN3 14
#define IN4 32

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

void setID() {
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  if(measure1.RangeStatus != 4) {
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(0);
  }
  Serial.print(",");
  
  if(measure2.RangeStatus != 4) {
    Serial.println(measure2.RangeMilliMeter);
  } else {
    Serial.println(0);
  }
}

void moveActuators(int ActuatorSpeed, int duration, bool forward) {
  if (forward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, ActuatorSpeed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, ActuatorSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, ActuatorSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, ActuatorSpeed);
  }
  
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    read_dual_sensors();
    delay(10);
  }
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println(F("Initializing sensors..."));
  setID();
}

void loop() {
  //read_dual_sensors();
  moveActuators(150, 30, true);
   delay(1);
  moveActuators(150, 30, false);
  delay(1);
}
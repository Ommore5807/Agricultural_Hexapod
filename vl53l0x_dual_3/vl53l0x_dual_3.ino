#include "Adafruit_VL53L0X.h"
#include <Wire.h>

// New I2C Addresses for sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Shutdown pins for VL53L0X sensors
#define SHT_LOX1 4
#define SHT_LOX2 5

// Create sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Measurement data storage
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// Function to reset and assign addresses to the sensors
void setID() {
  Serial.println(F("Resetting all sensors..."));
  
  // Step 1: Reset both sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(50);  // Ensure sensors fully reset

  // Step 2: Bring both sensors out of reset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(50);

  // Step 3: Activate LOX1 and keep LOX2 in reset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  delay(50);

  // Step 4: Initialize LOX1 with a new I2C address
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("ERROR: Failed to boot first VL53L0X!"));
    while (1);
  }
  Serial.println(F("VL53L0X #1 initialized!"));
  delay(50);

  // Step 5: Activate LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(50);

  // Step 6: Initialize LOX2 with a different I2C address
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("ERROR: Failed to boot second VL53L0X!"));
    while (1);
  }
  Serial.println(F("VL53L0X #2 initialized!"));
}

// Function to read both sensors and print results
void read_dual_sensors() {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  Serial.print(F("Sensor 1: "));
  if (measure1.RangeStatus != 4) {
    Serial.print(measure1.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" | Sensor 2: "));
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to open

  // Set I2C clock speed to 100kHz (more stable)
  Wire.begin();
  Wire.setClock(100000);

  // Initialize shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Initializing VL53L0X sensors..."));
  setID();  // Assign unique I2C addresses to sensors
}

void loop() {
  read_dual_sensors();
  delay(500);  // Read every 500ms
}

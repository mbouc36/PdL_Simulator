#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud

  // Wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");

  // Initialize the sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1); // Stay here if the sensor fails to initialize
  }

  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass 'true' for debug output

  if (measure.RangeStatus != 4) {  // 4 means out of range
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Out of range");
  }

  delay(100); // Delay between measurements
}
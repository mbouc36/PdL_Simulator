#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define NUM_SAMPLES 100

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int num_samples = 0;
int samples = 0;

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud

  // Wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X Avg Reading Test");

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

  if (measure.RangeStatus != 4 && num_samples < NUM_SAMPLES) {  // 4 means out of range

    num_samples++;
    samples += measure.RangeMilliMeter;
  } else if (num_samples >= NUM_SAMPLES){
    int average = samples/num_samples;
    Serial.print("Average Distance Measured: ");
    Serial.println(average);
  }
    else {
    Serial.println("Out of range");
  }

  delay(100); // Delay between measurements
}
#include <Wire.h>
#include <stdlib.h> 
#include <Adafruit_VL53L0X.h>

#define NUM_SAMPLES 100
enum Mode {
    IDLE,
    MEASURE
};

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int num_samples = 0;
int max_sample = 0;
int min_sample = 10000;
int samples = 0;
int position = 0;
float offset_sum;
int distance;
enum Mode mode = IDLE;

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

// Ask the user when to start reading
// Read avg/max/min at positing 1
// Ask the user if they would like to measure a new position

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

 
  if (mode == IDLE ){
    Serial.println(F("Would you like to measure a distance? (y/n) \n\n"));
    while (Serial.available() == 0) {
    // wait for input
    } 
    if (Serial.available() > 0){
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        if (input == 'y'){
            mode = MEASURE;
        } else if (input == 'n'){
            float average_offset = offset_sum/position;
            Serial.println(F("Average Offset Across all positions: \n\n"));
            Serial.println(average_offset);
            Serial.println(F("Offset measuring complete \n\n"));



        } else {
            Serial.println(F("Invalid input"));
            Serial.println(input);
        }
    }
  } else if (mode == MEASURE){
    lox.rangingTest(&measure, false); // pass 'true' for debug output
    if (measure.RangeStatus != 4 && num_samples < NUM_SAMPLES) {  // 4 means out of range
        num_samples++;
        distance = measure.RangeMilliMeter;
        samples += distance;
        max_sample = max(max_sample, distance);
        min_sample = min(min_sample, distance);

    } else if (num_samples >= NUM_SAMPLES){
        float average = samples/num_samples;
        Serial.print("Average Distance Measured: ");
        Serial.println(average);
        Serial.print("Max Distance Measured: ");
        Serial.println(max_sample);
        Serial.print("Min Distance Measured: ");
        Serial.println(min_sample);

        Serial.println(F("What is the physical measured distance? (number only) \n\n"));
        while (Serial.available() == 0) {
        // wait for input
        } 
        if (Serial.available() > 0){
            String input = Serial.readStringUntil('\n');
            input.trim();
            float offset = abs(average - input.toFloat());
            Serial.print("Offset Measured: ");
            Serial.println(offset);
            offset_sum += offset;
            position++;
            mode = IDLE;
        }

    } else {
        Serial.println("Out of range");
    }

    delay(100); // Delay between measurements
    } 
}
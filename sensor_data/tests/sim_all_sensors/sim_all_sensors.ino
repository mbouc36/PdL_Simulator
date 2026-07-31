/*
Author: Michael Boucouvalas
Date: July 3rd 2026
Description: Simulate printing all sensor data to the serial temrinal
*/


#define NUM_OUTPUT_VALUES 23
#define FREQUENCY 30 //Hz

String sensor_output = "0";
unsigned long now;
unsigned long last_print;
const unsigned long period_ms = 1000/FREQUENCY;


void setup() {
  Serial.begin(115200); // Start serial communication at 9600 baud

  // Wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  for (int i = 0; i < NUM_OUTPUT_VALUES -1 ; i++){
    sensor_output += " ,0";
  }

  if (sensor_output.length() != NUM_OUTPUT_VALUES){
    Serial.println("Invalid output size");
  }
}

void loop() {
  now = millis();

  if (now - last_print >= period_ms) {
    last_print = now;
    Serial.println(sensor_output);
  }

}
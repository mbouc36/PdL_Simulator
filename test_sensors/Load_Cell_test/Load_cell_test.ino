#include "HX711.h"

#define DOUT 3
#define CLK 2

HX711 scale;

float calibration_factor = -2150.0;   // better starting point from your data

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, CLK);

  if (!scale.is_ready()) {
    Serial.println("HX711 not ready");
    while (1);
  }

  Serial.println("Remove all load.");
  delay(3000);

  scale.set_scale();   // reset scale factor
  scale.tare();        // zero with no load

  Serial.println("Tare complete.");
  Serial.println("Now place the 500 g mass.");
  delay(3000);

  scale.set_scale(calibration_factor);
}

void loop() {
  long reading = scale.get_value(10);   // raw value after subtracting tare offset
  float grams  = scale.get_units(10);   // calibrated value in grams

  Serial.print("Net reading: ");
  Serial.print(reading);
  Serial.print(" | Weight: ");
  Serial.print(grams, 2);
  Serial.println(" g");

  delay(500);
}
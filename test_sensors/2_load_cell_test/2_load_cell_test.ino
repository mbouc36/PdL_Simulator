#include "HX711.h"

#define DOUT_BACK 5
#define CLK_BACK 4
#define DOUT_FRONT 3 
#define CLK_FRONT 2

#define INIT_NUM_RETRIES 10
#define INIT_RETRY_DELAY 1000

HX711 scale_front, scale_back;

float calibration_factor = -2150.0;   // better starting point from your data

void calibrate(HX711* scale){
  Serial.println("Remove all load");
  delay(3000);

  scale->set_scale();   // reset scale_front factor
  scale->tare();        // zero with no load

  Serial.println("Tare complete.");
  Serial.println("Now place the 500 g mass.");
  delay(5000);

  scale->set_scale(calibration_factor);
  Serial.println("Loaded calibration factor");
}

void setup() {
  Serial.begin(115200);
  scale_front.begin(DOUT_FRONT, CLK_FRONT);
  scale_back.begin(DOUT_BACK, CLK_BACK);


  if (!scale_front.wait_ready_retry(INIT_NUM_RETRIES, INIT_RETRY_DELAY)) {
    Serial.println("HX711 front not ready");
    while (1);
  }

  if (!scale_back.wait_ready_retry(INIT_NUM_RETRIES, INIT_RETRY_DELAY)) {
    Serial.println("HX711 back not ready");
    while (1);
  }

  Serial.println("Calibrate front scale");
  delay(3000);
  calibrate(&scale_front);

  Serial.println("Calibrate back scale");
  delay(3000);
  calibrate(&scale_back);

  Serial.println("Calibration complete");

}

void loop() {
  float grams_front  = scale_front.get_units(1);   // calibrated value in grams
  float grams_back  = scale_back.get_units(1);   // calibrated value in grams

  Serial.print("Weight: ");
  Serial.print(grams_front, 2); Serial.print("g, ");
  Serial.print(grams_back, 2);
  Serial.println(" g");

  //delay(500);
}
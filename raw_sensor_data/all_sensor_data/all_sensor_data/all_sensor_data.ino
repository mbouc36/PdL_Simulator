#include "HX711.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

unsigned long now;

// Load Cell
#define DOUT_BACK 5
#define CLK_BACK 4
#define DOUT_FRONT 3 
#define CLK_FRONT 2
HX711 scale_front, scale_back;

float calibration_factor = -2150.0; 
unsigned long lastLoadCell = 0;
const unsigned long HX711_PERIOD_MS = 100; // 10 Hz
float grams_front, grams_back;

// IMU 
LSM6 imu6_left, imu6_right;
LIS3MDL imu_mag_left, imu_mag_right;

float sensitivity = 4.375/ 1000;
unsigned long lastIMU = 0;
const unsigned long IMU_PERIOD_MS = 5; // 200 Hz

void calibrate_scale(HX711* scale){
  Serial.println("Remove all load");
  delay(3000);

  scale->set_scale();   
  scale->tare();       

  Serial.println("Tare complete.");
  Serial.println("Now place the 500 g mass.");
  delay(5000);

  scale->set_scale(calibration_factor);
  Serial.println("Loaded calibration factor");
}

void setup() {
  Serial.begin(115200);

  // IMU setup
  Wire.begin();
  /*LSM6::device_auto, LSM6::sa0_low*/
  if (!imu6_left.init(LSM6::device_auto, LSM6::sa0_high)) {
    Serial.println("Failed to detect left LSM6!");
    while (1);
  }

  if (!imu6_right.init()) {
    Serial.println("Failed to detect right LSM6!");
    while (1);
  }
  imu6_right.enableDefault();

  if (!imu_mag_left.init(LIS3MDL::device_auto, LIS3MDL::sa1_high)) {
    Serial.println("Failed to detect left LIS3MDL!");
    while (1);
  }
  imu_mag_left.enableDefault();
  imu6_left.writeReg(LSM6::CTRL2_G, (uint8_t) 0b01000010);


  if (!imu_mag_right.init()) {
    Serial.println("Failed to detect right LIS3MDL!");
    while (1);
  }
  imu_mag_right.enableDefault();
  imu6_right.writeReg(LSM6::CTRL2_G, (uint8_t) 0b01000010);

  // Load Cell Setup
  scale_front.begin(DOUT_FRONT, CLK_FRONT);
  scale_back.begin(DOUT_BACK, CLK_BACK);

  if (!scale_front.is_ready()) {
    Serial.println("HX711 front not ready");
    while (1);
  }

  if (!scale_back.is_ready()) {
    Serial.println("HX711 back not ready");
    while (1);
  }

  Serial.println("Calibrate front scale");
  delay(3000);
  calibrate_scale(&scale_front);

  Serial.println("Calibrate back scale");
  delay(3000);
  calibrate_scale(&scale_back);

  Serial.println("Calibration complete");

}

void loop() {
  now = millis();

  if (now - lastIMU >= IMU_PERIOD_MS) {
      lastIMU = now;
      imu6_left.read();
      imu_mag_left.read();
      imu6_right.read();
      imu_mag_right.read();
  }

  if (now - lastLoadCell >= HX711_PERIOD_MS) {
      lastLoadCell = now;
      grams_front  = scale_front.get_units(1);   
      grams_back  = scale_back.get_units(1);  
  }

  // Load Cell Prints
  Serial.print(grams_front, 2); Serial.print(", ");
  Serial.print(grams_back, 2);

  // Left IMU
  Serial.print(imu6_left.a.y); Serial.print(",");
  Serial.print(imu6_left.a.z); Serial.print(",");
  Serial.print(imu6_left.g.x * sensitivity); Serial.print(",");
  Serial.print(imu6_left.g.y * sensitivity); Serial.print(",");
  Serial.print(imu6_left.g.z * sensitivity); Serial.print(",");
  Serial.print(imu_mag_left.m.x); Serial.print(",");
  Serial.print(imu_mag_left.m.y); Serial.print(",");
  Serial.print(imu_mag_left.m.z);

  // Right IMU
  Serial.print(imu6_right.a.x); Serial.print(",");
  Serial.print(imu6_right.a.y); Serial.print(",");
  Serial.print(imu6_right.a.z); Serial.print(",");
  Serial.print(imu6_right.g.x * sensitivity); Serial.print(",");
  Serial.print(imu6_right.g.y * sensitivity); Serial.print(",");
  Serial.print(imu6_right.g.z * sensitivity); Serial.print(",");
  Serial.print(imu_mag_right.m.x); Serial.print(",");
  Serial.print(imu_mag_right.m.y); Serial.print(",");
  Serial.print(imu_mag_right.m.z);

  Serial.println("");

}
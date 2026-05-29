#include "HX711.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Adafruit_VL53L0X.h>

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

// TOF
#define XSHUT_1 6
#define XSHUT_2 7

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

const unsigned long TOF_PERIOD_MS = 33; // 30 Hz
unsigned long lastTOF = 0;
uint16_t distance1;
uint16_t distance2;

void calibrate_scale(HX711* scale){
  Serial.println("Remove all load");
  delay(3000);

  scale->set_scale();   
  scale->tare();       

  // Serial.println("Tare complete.");
  Serial.println("Now place the 500 g mass.");
  delay(5000);

  scale->set_scale(calibration_factor);
  Serial.println("Loaded calibration factor");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up");

  // IMU setup
  Wire.begin();
  /*LSM6::device_auto, LSM6::sa0_low*/
  if (!imu6_left.init(LSM6::device_auto, LSM6::sa0_high)) {
    Serial.println("Failed to detect left LSM6!");
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

  /* TOF Setup */ 
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Shut down both sensors
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);

  // Start sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(10);

  if (!lox1.begin(TOF1_ADDR)) {
    Serial.println(F("ERROR_SENSOR1_NOT_FOUND"));
    while (1);
  }

  // Start sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(10);

  if (!lox2.begin(TOF2_ADDR)) {
    Serial.println(F("ERROR_SENSOR2_NOT_FOUND"));
    while (1);
  }

  // // High accuracy mode
  // lox1.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  // lox2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  lox1.setMeasurementTimingBudgetMicroSeconds(20000);
  lox2.setMeasurementTimingBudgetMicroSeconds(20000);

  lox1.startRangeContinuous(33);
  lox2.startRangeContinuous(33);

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

  if (now - lastTOF >= TOF_PERIOD_MS) {
      lastTOF = now;
      distance1 = lox1.readRange();
      distance2 = lox2.readRange();
  }

  // Load Cell Prints
  Serial.print(grams_front, 2); Serial.print(", ");
  Serial.print(grams_back, 2);

  // TOF Prints
  Serial.print(distance1);
  Serial.print(distance2);

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
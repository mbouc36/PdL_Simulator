#include "HX711.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <VL53L0X.h>


#define FREQUENCY 30
unsigned long now;
unsigned long last_print;
const unsigned long period_ms = 1000/FREQUENCY;

// Load Cell
#define DOUT_BACK 5
#define CLK_BACK 4
#define DOUT_FRONT 3
#define CLK_FRONT 2
#define INIT_NUM_RETRIES 10
#define INIT_RETRY_DELAY 1000

HX711 scale_front, scale_back;

float calibration_factor = -2150.0; 

// IMU 
LSM6 imu6_left, imu6_right;
LIS3MDL imu_mag_left, imu_mag_right;

float sensitivity = 4.375/ 1000;

// TOF
#define XSHUT_1 6
#define XSHUT_2 7

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31

VL53L0X lox_left, lox_right;


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
  Wire.begin();

  // IMU setup
  /*LSM6::device_auto, LSM6::sa0_low*/
  if (!imu6_left.init(LSM6::device_auto, LSM6::sa0_high)) {
    Serial.println("Failed to detect left LSM6!");
  }
  imu6_left.enableDefault();

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

  Serial.println("Completed IMU setup");

  /* TOF Setup */ 
  Serial.println("Starting TOF setup");
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Shut down both sensors
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);

  // Start sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(10);

  lox_left.setTimeout(500);
  if (!lox_left.init())
  {
    Serial.println("Failed to detect and initialize lox_left!");
    while (1) {}
  }
  lox_left.setAddress(TOF1_ADDR);

  // Start sensor 1
  digitalWrite(XSHUT_2, HIGH);
  delay(10);

  lox_right.setTimeout(500);
  if (!lox_right.init())
  {
    Serial.println("Failed to detect and initialize lox_right!");
    while (1) {}
  }
  lox_right.setAddress(TOF2_ADDR);

  lox_left.startContinuous();
  lox_right.startContinuous();
  Serial.println("Finished TOF setup");

  // Load Cell Setup
  scale_front.begin(DOUT_FRONT, CLK_FRONT);
  if (!scale_front.wait_ready_retry(INIT_NUM_RETRIES, INIT_RETRY_DELAY)) {
    Serial.println("HX711 front not ready");
    while (1);
  }

  scale_back.begin(DOUT_BACK, CLK_BACK);
  if (!scale_back.wait_ready_retry(INIT_NUM_RETRIES, INIT_RETRY_DELAY)) {
    Serial.println("HX711 back not ready");
    while (1);
  }
  Serial.println("Calibrate back scale");
  delay(3000);
  calibrate_scale(&scale_back);

  Serial.println("Calibrate front scale");
  delay(3000);
  calibrate_scale(&scale_front);
  Serial.print("Ready");
}

void loop() {
  now = millis();

  if (now - last_print >= period_ms) {
    last_print = now;
    imu6_left.read();
    imu_mag_left.read();
    imu6_right.read();
    imu_mag_right.read();

    // Time
    Serial.print(now); Serial.print(", ");

    // Load Cell Prints
    Serial.print(scale_front.get_units(1), 2); Serial.print(", ");
    Serial.print(scale_back.get_units(1), 2); Serial.print(", ");

    // TOF Prints
    Serial.print(lox_left.readRangeContinuousMillimeters()); Serial.print(", ");
    Serial.print(lox_right.readRangeContinuousMillimeters()); Serial.print(", ");

    // Left IMU
    Serial.print(imu6_left.a.x); Serial.print(", ");
    Serial.print(imu6_left.a.y); Serial.print(", ");
    Serial.print(imu6_left.a.z); Serial.print(", ");
    Serial.print(imu6_left.g.x * sensitivity); Serial.print(", ");
    Serial.print(imu6_left.g.y * sensitivity); Serial.print(", ");
    Serial.print(imu6_left.g.z * sensitivity); Serial.print(", ");
    Serial.print(imu_mag_left.m.x); Serial.print(", ");
    Serial.print(imu_mag_left.m.y); Serial.print(", ");
    Serial.print(imu_mag_left.m.z); Serial.print(", ");

    // Right IMU
    Serial.print(imu6_right.a.x); Serial.print(", ");
    Serial.print(imu6_right.a.y); Serial.print(", ");
    Serial.print(imu6_right.a.z); Serial.print(", ");
    Serial.print(imu6_right.g.x * sensitivity); Serial.print(", ");
    Serial.print(imu6_right.g.y * sensitivity); Serial.print(", ");
    Serial.print(imu6_right.g.z * sensitivity); Serial.print(", ");
    Serial.print(imu_mag_right.m.x); Serial.print(", ");
    Serial.print(imu_mag_right.m.y); Serial.print(", ");
    Serial.print(imu_mag_right.m.z);

    Serial.println("");
  }

}
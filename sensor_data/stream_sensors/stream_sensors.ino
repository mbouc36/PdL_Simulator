#include "HX711.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <VL53L1X.h>

struct Position{
  float x;
  float y;
  float z;
};

struct IMUData{
  Position a;
  Position g;
  Position m;
};

struct Snapshot{
  IMUData left_imu;
  IMUData right_imu;
  int left_distance;
  int right_distance;
  int front_weight;
  int back_weight;
};


#define PRINT_FREQUENCY 30 //Hz
unsigned long now;
unsigned long last_print;
const unsigned long period_ms = 1000/PRINT_FREQUENCY;
Snapshot latest_snapshot;
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

VL53L1X lox_left, lox_right;


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

  lox_left.startContinuous(10);
  lox_right.startContinuous(10);
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
    Serial.print(now);

    // Load Cells
    Serial.print(latest_snapshot.front_weight);
    Serial.print(latest_snapshot.back_weight);

    // TOF
    Serial.print(latest_snapshot.left_distance);
    Serial.print(latest_snapshot.right_distance);

    // IMU
    Serial.print(latest_snapshot.left_imu.a.x);
    Serial.print(latest_snapshot.left_imu.a.y);
    Serial.print(latest_snapshot.left_imu.a.z);
    Serial.print(latest_snapshot.left_imu.g.x);
    Serial.print(latest_snapshot.left_imu.g.y);
    Serial.print(latest_snapshot.left_imu.g.z);
    Serial.print(latest_snapshot.left_imu.m.x);
    Serial.print(latest_snapshot.left_imu.m.y);
    Serial.print(latest_snapshot.left_imu.m.z);

    Serial.print(latest_snapshot.right_imu.a.x);
    Serial.print(latest_snapshot.right_imu.a.y);
    Serial.print(latest_snapshot.right_imu.a.z);
    Serial.print(latest_snapshot.right_imu.g.x);
    Serial.print(latest_snapshot.right_imu.g.y);
    Serial.print(latest_snapshot.right_imu.g.z);
    Serial.print(latest_snapshot.right_imu.m.x);
    Serial.print(latest_snapshot.right_imu.m.y);
    Serial.print(latest_snapshot.right_imu.m.z);

  }

  if (imu6_left.readReg((LSM6::STATUS_REG) & 0b00000011) && (imu_mag_left.readReg(LIS3MDL::STATUS_REG) & 0b00001111)){
    latest_snapshot.left_imu.a.x = imu6_left.a.x;
    latest_snapshot.left_imu.a.y = imu6_left.a.y;
    latest_snapshot.left_imu.a.z = imu6_left.a.z;
    latest_snapshot.left_imu.g.x = imu6_left.g.x;
    latest_snapshot.left_imu.g.y = imu6_left.g.y;
    latest_snapshot.left_imu.g.z = imu6_left.g.z;
    latest_snapshot.left_imu.m.x = imu_mag_left.m.x;
    latest_snapshot.left_imu.m.y = imu_mag_left.m.y;
    latest_snapshot.left_imu.m.z = imu_mag_left.m.z;
  }

  if (imu6_right.readReg((LSM6::STATUS_REG) & 0b00000011) && (imu_mag_right.readReg(LIS3MDL::STATUS_REG) & 0b00001111)){
    latest_snapshot.right_imu.a.x = imu6_right.a.x;
    latest_snapshot.right_imu.a.y = imu6_right.a.y;
    latest_snapshot.right_imu.a.z = imu6_right.a.z;
    latest_snapshot.right_imu.g.x = imu6_right.g.x;
    latest_snapshot.right_imu.g.y = imu6_right.g.y;
    latest_snapshot.right_imu.g.z = imu6_right.g.z;
    latest_snapshot.right_imu.m.x = imu_mag_right.m.x;
    latest_snapshot.right_imu.m.y = imu_mag_right.m.y;
    latest_snapshot.right_imu.m.z = imu_mag_right.m.z;
  }

  if (lox_left.dataReady()){
    latest_snapshot.left_distance = lox_left.read(false);
  }

  if (lox_right.dataReady()){
    latest_snapshot.right_distance = lox_right.read(false);
  } 

  if (scale_front.is_ready()){
    latest_snapshot.front_weight = scale_front.get_units(1);
  }

  if (scale_back.is_ready()){
    latest_snapshot.back_weight = scale_back.get_units(1);
  }

}
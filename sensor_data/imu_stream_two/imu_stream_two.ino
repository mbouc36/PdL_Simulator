#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 imu6_left, imu6_right;
LIS3MDL imu_mag_left, imu_mag_right;

float sensitivity = 4.375/ 1000;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Settig upt left IMU");
  /*LSM6::device_auto, LSM6::sa0_low*/
  if (!imu6_left.init(LSM6::device_auto, LSM6::sa0_high)) {
    Serial.println("Failed to detect left LSM6!");
    while (1);
  }
  Serial.println("Finished setting up left gyro");
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
}

void loop() {
  imu6_left.read();
  imu_mag_left.read();
  imu6_right.read();
  imu_mag_right.read();

  Serial.print(imu6_left.a.x); Serial.print(",");
  Serial.print(imu6_left.a.y); Serial.print(",");
  Serial.print(imu6_left.a.z); Serial.print(",");
  Serial.print(imu6_left.g.x * sensitivity); Serial.print(",");
  Serial.print(imu6_left.g.y * sensitivity); Serial.print(",");
  Serial.print(imu6_left.g.z * sensitivity); Serial.print(",");
  // Serial.print(imu6_left.g.x); Serial.print(",");
  // Serial.print(imu6_left.g.y); Serial.print(",");
  // Serial.print(imu6_left.g.z); Serial.print(",");
  Serial.print(imu_mag_left.m.x); Serial.print(",");
  Serial.print(imu_mag_left.m.y); Serial.print(",");
  Serial.print(imu_mag_left.m.z);

  Serial.print(imu6_right.a.x); Serial.print(",");
  Serial.print(imu6_right.a.y); Serial.print(",");
  Serial.print(imu6_right.a.z); Serial.print(",");
  Serial.print(imu6_right.g.x * sensitivity); Serial.print(",");
  Serial.print(imu6_right.g.y * sensitivity); Serial.print(",");
  Serial.print(imu6_right.g.z * sensitivity); Serial.print(",");
  // Serial.print(imu6_right.g.x); Serial.print(",");
  // Serial.print(imu6_right.g.y); Serial.print(",");
  // Serial.print(imu6_right.g.z); Serial.print(",");
  Serial.print(imu_mag_right.m.x); Serial.print(",");
  Serial.print(imu_mag_right.m.y); Serial.print(",");
  Serial.println(imu_mag_right.m.z);

}


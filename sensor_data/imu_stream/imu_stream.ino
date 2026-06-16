#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 imu6;
LIS3MDL imu_mag;

float sensitivity = 4.375/ 1000;
unsigned long last_print = 0;
unsigned long dt;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  /*LSM6::device_auto, LSM6::sa0_low*/
  if (!imu6.init(LSM6::device_auto, LSM6::sa0_high)) {
    Serial.println("Failed to detect LSM6!");
    while (1);
  }
  imu6.enableDefault(); 


  if (!imu_mag.init(LIS3MDL::device_auto, LIS3MDL::sa1_high)) {
    Serial.println("Failed to detect left LIS3MDL!");
    while (1);
  }
  imu_mag.enableDefault();
  imu6.writeReg(LSM6::CTRL2_G, (uint8_t) 0b01000010);

}

void loop() {
  imu6.read();
  imu_mag.read();


  Serial.print(millis()); Serial.print(",");
  Serial.print(imu6.a.x); Serial.print(",");
  Serial.print(imu6.a.y); Serial.print(",");
  Serial.print(imu6.a.z); Serial.print(",");
  Serial.print(imu6.g.x * sensitivity); Serial.print(",");
  Serial.print(imu6.g.y * sensitivity); Serial.print(",");
  Serial.print(imu6.g.z * sensitivity); Serial.print(",");
  Serial.print(imu_mag.m.x); Serial.print(",");
  Serial.print(imu_mag.m.y); Serial.print(",");
  Serial.print(imu_mag.m.z);
  Serial.println("");


}


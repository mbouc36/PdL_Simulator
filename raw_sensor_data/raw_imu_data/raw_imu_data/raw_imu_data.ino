#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 imu6;
LIS3MDL imuMag;

float sensitivity = 4.375/ 1000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!imu6.init()) {
    Serial.println("Failed to detect LSM6!");
    while (1);
  }
  imu6.enableDefault();

  if (!imuMag.init()) {
    Serial.println("Failed to detect LIS3MDL!");
    while (1);
  }
  imuMag.enableDefault();
  imu6.writeReg(LSM6::CTRL2_G, (uint8_t) 0b01000010);
}

void loop() {
  imu6.read();
  imuMag.read();

  Serial.print(imu6.a.x); Serial.print(",");
  Serial.print(imu6.a.y); Serial.print(",");
  Serial.print(imu6.a.z); Serial.print(",");
  Serial.print(imu6.g.x * sensitivity); Serial.print(",");
  Serial.print(imu6.g.y * sensitivity); Serial.print(",");
  Serial.print(imu6.g.z * sensitivity); Serial.print(",");
  Serial.print(imuMag.m.x); Serial.print(",");
  Serial.print(imuMag.m.y); Serial.print(",");
  Serial.println(imuMag.m.z);

  //delay(100);
}


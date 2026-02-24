#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 imu6;
LIS3MDL imuMag;

void setup() {
  Serial.begin(9600);
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
}

void loop() {
  imu6.read();
  imuMag.read();

  // Print Accel

  Serial.print("A_X:");
  Serial.print(imu6.a.x);
  Serial.print(" ");
  Serial.print("A_Y:");
  Serial.print(imu6.a.y);
  Serial.print(" ");
  Serial.print("A_Z:");
  Serial.println(imu6.a.z);

  // Print Gyro
  Serial.print("G_X:");
  Serial.print(imu6.g.x);
  Serial.print(" ");
  Serial.print("G_Y:");
  Serial.print(imu6.g.y);
  Serial.print(" ");
  Serial.print("G_Z:");
  Serial.println(imu6.g.z);

  // // Print Mag allo
  Serial.print("M_X:");
  Serial.print(imuMag.m.x);
  Serial.print(" ");
  Serial.print("M_Y:");
  Serial.print(imuMag.m.y);
  Serial.print(" ");
  Serial.print("M_Z:");
  Serial.println(imuMag.m.z);
  
  Serial.println();
  delay(100);
}


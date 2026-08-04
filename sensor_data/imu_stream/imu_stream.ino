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
  if (!imu6.init()) {
    Serial.println("Failed to detect LSM6!");
    while (1);
  }
  imu6.enableDefault(); 


  if (!imu_mag.init()) {
    Serial.println("Failed to detect left LIS3MDL!");
    while (1);
  }
  imu_mag.enableDefault();
  imu6.writeReg(LSM6::CTRL2_G, (uint8_t) 0b01000010);
  imu_mag.writeReg(LIS3MDL::CTRL_REG1, 0b00011100); // Set mag to 80Hz


}

void loop() {
   if (imu6.readReg((LSM6::STATUS_REG) & 0b00000011) && (imu_mag.readReg(LIS3MDL::STATUS_REG) & 0b00001111)){
      imu6.read();
      imu_mag.read();
      


      // Serial.print(millis()); Serial.print(",");
      // Serial.print(imu6.a.x); Serial.print(",");
      // Serial.print(imu6.a.y); Serial.print(",");
      // Serial.print(imu6.a.z); Serial.print(",");
      // Serial.print(imu6.g.x * sensitivity); Serial.print(",");
      // Serial.print(imu6.g.y * sensitivity); Serial.print(",");
      // Serial.print(imu6.g.z * sensitivity); Serial.print(",");
      Serial.print(imu_mag.m.x); Serial.print(",");
      Serial.print(imu_mag.m.y); Serial.print(",");
      Serial.print(imu_mag.m.z);
      Serial.println("");
  }

}


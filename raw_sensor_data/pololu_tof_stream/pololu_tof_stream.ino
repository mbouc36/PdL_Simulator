#include <Wire.h>
#include <VL53L0X.h>

VL53L0X lox_left, lox_right;

#define XSHUT_1 6
#define XSHUT_2 7

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31

#define SAMPLE_PERIOD 0


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

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

  // Configure for long range
  // lower the return signal rate limit (default is 0.25 MCPS)
  // lox_left.setSignalRateLimit(0.1);
  // // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  // lox_left.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // lox_left.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);


  // // Configure for long range
  // // lower the return signal rate limit (default is 0.25 MCPS)
  // lox_right.setSignalRateLimit(0.1);
  // // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  // lox_right.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  // lox_right.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // Set for conntiuous mode
  lox_left.startContinuous(SAMPLE_PERIOD);
  lox_right.startContinuous(SAMPLE_PERIOD);
}

void loop() {
  Serial.print(F("TOF1:"));
  Serial.print((float) lox_left.readRangeContinuousMillimeters());
  Serial.print(", ");
  Serial.print(F("TOF2:"));
  Serial.print((float) lox_right.readRangeContinuousMillimeters());
  Serial.println();

}


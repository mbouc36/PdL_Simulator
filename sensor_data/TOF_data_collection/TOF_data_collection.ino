//Stream raw raws to the python TOF_stream.py script 


#include <Wire.h>
#include <Adafruit_VL53L0X.h>
//#include <vl53l0x-arduino.h>

#define XSHUT_1 6
#define XSHUT_2 7

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31

#define FREQUENCY 30
unsigned long now;
unsigned long last_print = 0;
const unsigned long period_ms = 1000/FREQUENCY;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

void setup() {
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

  Serial.println(F("READY"));
}

void loop() {
  now = millis();

  if (now - last_print >= period_ms) {
    last_print = now;
    uint16_t distance1 = lox1.readRange();
    uint16_t distance2 = lox2.readRange();

    Serial.print(now); Serial.print(", ");
    Serial.print(distance1); Serial.print(", ");
    Serial.println(distance2);
  }
}
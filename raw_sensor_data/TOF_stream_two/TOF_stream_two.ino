%Stream raw raws to the python TOF_stream.py script 


#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define XSHUT_1 2
#define XSHUT_2 3

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
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

  // High accuracy mode
  lox1.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  lox2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);

  Serial.println(F("READY"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;

  int distance1 = -1;
  int distance2 = -1;

  // Read sensor 1
  lox1.rangingTest(&measure1, false);

  if (measure1.RangeStatus != 4) {
    distance1 = measure1.RangeMilliMeter;
  }

  // Read sensor 2
  lox2.rangingTest(&measure2, false);

  if (measure2.RangeStatus != 4) {
    distance2 = measure2.RangeMilliMeter;
  }

  // Stream both values on one line
  Serial.print(F("TOF1:"));
  Serial.print(distance1);

  Serial.print(F(",TOF2:"));
  Serial.println(distance2);

  delay(50);
}
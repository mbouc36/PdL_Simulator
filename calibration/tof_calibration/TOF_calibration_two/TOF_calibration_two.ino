%Ardiuno script to take 100 measuremnts at 10 known distances 

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define NUM_SAMPLES 100
#define MAX_VALID_DISTANCE_MM 375

#define XSHUT_1 2
#define XSHUT_2 3

#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

const int NUM_CAL_POINTS = 10;

const int actualDistances[NUM_CAL_POINTS] = {
  50, 100, 150, 175, 200,
  225, 250, 275, 300, 325
};

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  Serial.println(F("Dual VL53L0X Calibration Data Collection"));
  delay(5000);

  setupSensors();

  Serial.println(F("CALIBRATION_DATA_START"));
  Serial.println(F("actual_mm,sensor1_measured_mm,sensor2_measured_mm"));

  for (int i = 0; i < NUM_CAL_POINTS; i++) {
    Serial.println();
    Serial.print(F("Place target at "));
    Serial.print(actualDistances[i]);
    Serial.println(F(" mm"));

    Serial.println(F("Waiting 5 seconds..."));
    delay(5000);

    float avg1 = getAverageMeasurement(lox1, 1);
    float avg2 = getAverageMeasurement(lox2, 2);

    Serial.print(F("CAL_DATA,"));
    Serial.print(actualDistances[i]);
    Serial.print(F(","));
    Serial.print(avg1, 6);
    Serial.print(F(","));
    Serial.println(avg2, 6);
  }

  Serial.println(F("CALIBRATION_DATA_END"));
  Serial.println(F("Calibration data collection complete."));
  Serial.println(F("Python will compute the polynomial coefficients."));

  while (1);
}

void loop() {
}

void setupSensors() {
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);

  digitalWrite(XSHUT_1, HIGH);
  delay(10);

  if (!lox1.begin(TOF1_ADDR)) {
    Serial.println(F("Failed to boot VL53L0X sensor 1"));
    while (1);
  }

  digitalWrite(XSHUT_2, HIGH);
  delay(10);

  if (!lox2.begin(TOF2_ADDR)) {
    Serial.println(F("Failed to boot VL53L0X sensor 2"));
    while (1);
  }

  lox1.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  lox2.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);

  Serial.println(F("Both VL53L0X sensors initialized."));
  Serial.print(F("Sensor 1 address: 0x"));
  Serial.println(TOF1_ADDR, HEX);
  Serial.print(F("Sensor 2 address: 0x"));
  Serial.println(TOF2_ADDR, HEX);
}

float getAverageMeasurement(Adafruit_VL53L0X &sensor, int sensorID) {
  VL53L0X_RangingMeasurementData_t measure;

  long sum = 0;
  int validSamples = 0;

  while (validSamples < NUM_SAMPLES) {
    sensor.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      int distance = measure.RangeMilliMeter;

      if (distance <= MAX_VALID_DISTANCE_MM) {
        sum += distance;
        validSamples++;

        Serial.print(F("S"));
        Serial.print(sensorID);
        Serial.print(F(" sample "));
        Serial.print(validSamples);
        Serial.print(F("/"));
        Serial.print(NUM_SAMPLES);
        Serial.print(F(": "));
        Serial.print(distance);
        Serial.println(F(" mm"));
      } else {
        Serial.print(F("S"));
        Serial.print(sensorID);
        Serial.print(F(" rejected: "));
        Serial.print(distance);
        Serial.println(F(" mm"));
      }
    } else {
      Serial.print(F("S"));
      Serial.print(sensorID);
      Serial.println(F(" out of range"));
    }

    delay(50);
  }

  return (float)sum / validSamples;
}
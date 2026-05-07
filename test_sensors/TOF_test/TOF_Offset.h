
#ifndef TOF_OFFSET
#define TOF_OFFSET

#include <Wire.h>
#include <stdlib.h> 
#include <Adafruit_VL53L0X.h>

#include "test_sensors/Offset_Test_Template.h"

class TOFOffset: public OffsetTest {

private:
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
    VL53L0X_RangingMeasurementData_t measure;

public:

    void init() override {
        Serial.println("VL53L0X Offset Test");
        // Initialize the sensor
        if (!lox.begin()) {
            Serial.println(F("Failed to boot VL53L0X"));
            while (1); // Stay here if the sensor fails to initialize
        }
    }

    int read_sensor() override {
        lox.rangingTest(&measure, false); // pass 'true' for debug output
        if (measure.RangeStatus != 4) {  // 4 means out of range
            return measure.RangeMilliMeter;
        } else {
            return -1;
        }
    }


};
#endif
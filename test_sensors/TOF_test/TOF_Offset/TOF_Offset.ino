#include "TOF_Offset.h"


TOFOffset TOFOffsetTest;

void setup(){
    Serial.begin(9600); // Start serial communication at 9600 baud

    // Wait until serial port opens for native USB devices
    while (!Serial) {
        delay(1);
    }
    TOFOffsetTest.init();
}

void loop(){
    TOFOffsetTest.detect_offset();
}
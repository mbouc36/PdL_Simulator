#include <Arduino.h>
#include "TOF_Offset.h"

TOFOffset TOFOffsetTest;

void setup(){
    TOFOffsetTest.init();
}

void loop(){
    TOFOffsetTest.detect_offset();
}
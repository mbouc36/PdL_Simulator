#include <Wire.h>
#include <stdlib.h> 

#ifndef OFFSET_TEST
#define OFFSET_TEST

class OffsetTest{

#define NUM_SAMPLES 100
#define MAX_NUM_SAMPLE 1000000
enum Mode {
    IDLE,
    MEASURE
};

private:
    int num_samples;
    int max_sample;
    int min_sample;
    int samples;
    int position = 0;
    float offset_sum;
    int measurement;
    enum Mode mode = IDLE;

public:

    OffsetTest(){

    }

    virtual void init(){
        /* Initialize function for respective sensor */
    }

    virtual float read_sensor(){
        /* Get reading from the respective sensor */
    }

    void detect_offset() {
        init();

        if (mode == IDLE){
            num_samples = 0;
            max_sample = 0;
            min_sample = MAX_NUM_SAMPLE;
            samples = 0;
            Serial.println(F("Would you like to measure a value? (y/n) \n\n"));
            while (Serial.available() == 0) {
            // wait for input
            } 
            if (Serial.available() > 0){
                String input = Serial.readStringUntil('\n');
                input.trim();
                input.toLowerCase();
                if (input == "y"){
                    Serial.println(F("Starting measurement \n\n"));
                    mode = MEASURE;
                } else if (input == "n"){
                    float average_offset = offset_sum/position;
                    Serial.println(F("Average Offset Across all positions: \n\n"));
                    Serial.println(average_offset);
                    Serial.println(F("Offset measuring complete \n\n"));
                    while (true);

                } else {
                    Serial.println(F("Invalid input"));
                    Serial.println(input);
                }
            }
        } else if (mode == MEASURE){
            if (num_samples < NUM_SAMPLES) {  
                measurement = read_sensor();
                // if an invalid measurement is read 
                if (measurement == -1) {
                    Serial.print("Error or Out of Range, trying again");
                    return;
                }
                num_samples++;
                samples += measurement;
                max_sample = max(max_sample, measurement);
                min_sample = min(min_sample, measurement);

            } else if (num_samples >= NUM_SAMPLES){
                float average = samples/num_samples;
                Serial.print("Average Value Measured: ");
                Serial.println(average);
                Serial.print("Max Value Measured: ");
                Serial.println(max_sample);
                Serial.print("Min Value Measured: ");
                Serial.println(min_sample);

                Serial.println(F("What is the physical measurement? (number) \n\n"));
                while (Serial.available() == 0) {
                // wait for input
                } 
                if (Serial.available() > 0){
                    String input = Serial.readStringUntil('\n');
                    input.trim();
                    float offset = abs(average - input.toFloat());
                    Serial.print("Offset Measured: ");
                    Serial.println(offset);
                    offset_sum += offset;
                    position++;
                    mode = IDLE;
                }

            } else {
                Serial.println("Out of range");
            }

            delay(100); // Delay between measurements
            } 
        }
};
#endif
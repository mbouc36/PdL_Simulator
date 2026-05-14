#include <LSM6.h>
#include <LIS3MDL.h>
#include <Wire.h>


// Smoothing factor
#define ALPHA 1
#define EPSILON .00001

float AxRaw,AyRaw,AzRaw,GxRaw,GyRaw,GzRaw,MxRaw,MyRaw,MzRaw;
float AxCal,AyCal,AzCal,GxCal,GyCal,GzCal,MxCal,MyCal,MzCal;
float rollA,pitchA,yawM,rollC,pitchC,yawC;
float rollG = 0;
float pitchG = 0 ;
float yawG = 0;
float deltaRoll = 0;
float deltaPitch = 0;
float deltaYaw = 0;
float alpha = ALPHA;
int tStart =millis();

LSM6 imu6;
LIS3MDL imuMag;

void setup() {
    Serial.begin(115200);
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

void calibrateSensors() {
    const float axOffset = 456.30506221287214;
    const float ayOffset = 150.86898594905324;
    const float azOffset = 114.94042002821243;
    const float axScale = 5.96793176747815e-05;
    const float ayScale = 5.96793176747815e-05;
    const float azScale = 5.96793176747815e-05;
    const float gxOffset = -2977.3751823973225;
    const float gyOffset = -339.4774936150127;
    const float gzOffset = 5290.978759135588;
    const float mxOffset = 68.58243094557884;
    const float myOffset = 587.0414406456362;
    const float mzOffset = 1209.6904829957984;
    const float mxScale = 0.0002627662002515864;
    const float myScale = 0.00024597451981347936;
    const float mzScale = 0.00028647685730964306;

    AxCal = (AxRaw - axOffset) * axScale;
    AyCal = (AyRaw - ayOffset) * ayScale;
    AzCal = (AzRaw - azOffset) * azScale;
    GxCal = GxRaw*180/PI - gxOffset;
    GyCal = GyRaw*180/PI - gyOffset;
    GzCal = GzRaw*180/PI - gzOffset;
    MxCal = (MxRaw - mxOffset) * mxScale;
    MyCal = (MyRaw - myOffset) * myScale;
    MzCal = (MzRaw - mzOffset) * mzScale;
}



void loop() {
    imu6.read();
    imuMag.read();

    AxRaw = imu6.a.x;
    AyRaw = imu6.a.y;
    AzRaw = imu6.a.z;

    GxRaw = imu6.g.x;
    GyRaw = imu6.g.y;
    GzRaw = imu6.g.z;

    MxRaw= imuMag.m.x;
    MyRaw= imuMag.m.y;
    MzRaw= imuMag.m.z;

    calibrateSensors();

    rollA = atan2(AyCal, sqrt(AzCal*AzCal + AxCal*AxCal))*180/PI;
    pitchA = atan2(AxCal,sqrt(AzCal*AzCal + AyCal*AyCal))*180/PI;

    deltaRoll = GxCal*(millis()-tStart)/1000.;
    deltaPitch = -GyCal*(millis()-tStart)/1000.;
    deltaYaw = -GzCal*(millis()-tStart)/1000.;
    tStart = millis();

    rollG = rollG + deltaRoll;
    pitchG = pitchG + deltaPitch;
    yawG = yawG + deltaYaw;

    yawM = atan2(MyCal,MxCal)*180./PI;

    rollC = alpha*(rollA) + (1-alpha)*(rollC+deltaRoll);
    pitchC = alpha*(pitchA) + (1-alpha)*(pitchC+deltaPitch);
    yawC = alpha*(yawM) + (1-alpha)*(yawC + deltaYaw);

    Serial.print("yawC:");Serial.print(yawC);Serial.print(',');
    Serial.print("yawM:");Serial.print(yawM);Serial.print(',');
    Serial.print("yawG:");Serial.print(yawG);Serial.print(',');
    Serial.print("LL:");Serial.print(-90);Serial.print(',');
    Serial.print("UL:");Serial.println(90);


    Serial.print("rollC:");Serial.print(rollC);Serial.print(',');
    Serial.print("pitchC:");Serial.print(pitchC);Serial.print(',');
    Serial.print("yawC:");Serial.print(yawC);Serial.print(',');
    Serial.print("LL:");Serial.print(-90);Serial.print(',');
    Serial.print("UL:");Serial.println(90);

    delay(100);
}
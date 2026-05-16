#include <LSM6.h>
#include <LIS3MDL.h>
#include <Wire.h>


// Smoothing factor
#define ALPHA .1
#define EPSILON .00001

// Registers 
#define ACCELEROMETER_RANGE_REGISTER LSM6::CTRL1_XL
#define GYRO_RANGE_REGISTER LSM6::CTRL2_G

#define SENSITIVITY_IN_MDPS 4.375

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
float sensitivity_factor = SENSITIVITY_IN_MDPS / 1000;
unsigned long tStart = millis();
unsigned long now, dt;

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
    // uint8_t regValue = imu6.readReg( LSM6::CTRL2_G);
    // Serial.print("The value of the register is: ");
    // Serial.println(regValue);

    imu6.writeReg(GYRO_RANGE_REGISTER, 0b01000010);
    tStart = millis();
}

void calibrateSensors() {
    const float axOffset = -150.68299529555588;
    const float ayOffset = -271.90359039723444;
    const float azOffset = 583.9725043510443;
    const float axScale = 6.007457014437071e-05;
    const float ayScale = 6.007457014437071e-05;
    const float azScale = 6.007457014437071e-05;
    const float gxOffset =  -0.29664999999999997;
    const float gyOffset =  -0.22954999999999998;
    const float gzOffset = 0.10610000000000001;
    const float mxOffset = -2688.4487689349876;
    const float myOffset = 1529.2091567652617;
    const float mzOffset = 1299.411256472524;
    const float mxScale = 0.0002780708679958069;
    const float myScale = 0.00025073835117261613;
    const float mzScale = 0.00028245341970921926;


    AxCal = (AxRaw - axOffset) * axScale;
    AyCal = (AyRaw - ayOffset) * ayScale;
    AzCal = (AzRaw - azOffset) * azScale;
    GxCal = GxRaw - gxOffset;
    GyCal = GyRaw - gyOffset;
    GzCal = GzRaw - gzOffset;
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

    GxRaw = imu6.g.x * sensitivity_factor;
    GyRaw = imu6.g.y * sensitivity_factor;
    GzRaw = imu6.g.z * sensitivity_factor;

    MxRaw= imuMag.m.x;
    MyRaw= imuMag.m.y;
    MzRaw= imuMag.m.z;
    
    now = millis(); 
    dt = (now - tStart) / 1000.;
    tStart = now;

    calibrateSensors();

    // rollA = atan2(AyCal, sqrt(AzCal*AzCal + AxCal*AxCal))*180/PI;
    // pitchA = atan2(AxCal,sqrt(AzCal*AzCal + AyCal*AyCal))*180/PI;


    deltaRoll = GxCal*dt;
    deltaPitch = -GyCal*dt;
    deltaYaw = -GzCal*dt;

    // rollG = rollG + deltaRoll;
    // pitchG = pitchG + deltaPitch;
    // yawG = yawG + deltaYaw;

    // yawM = atan2(MyCal,MxCal)*180./PI;

    rollA = atan2(AyCal, AzCal) * 180.0 / PI;
    pitchA = atan2(-AxCal, sqrt(AyCal*AyCal + AzCal*AzCal)) * 180.0 / PI;

    float phi = rollA * PI / 180.0;
    float theta = pitchA * PI / 180.0;

    float mxH = MxCal*cos(theta)
            + MyCal*sin(phi)*sin(theta)
            + MzCal*cos(phi)*sin(theta);

    float myH = MyCal*cos(phi) - MzCal*sin(phi);

    yawM = atan2(myH, mxH) * 180.0 / PI;

    rollC = alpha*(rollA) + (1-alpha)*(rollC+deltaRoll);
    pitchC = alpha*(pitchA) + (1-alpha)*(pitchC+deltaPitch);
    
    float yawGyro = yawC + deltaYaw;
    float yawError = yawM - yawGyro;
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;
    yawC = yawGyro + alpha * yawError;
    // keep yaw in -180 to 180
    if (yawC > 180) yawC -= 360;
    if (yawC < -180) yawC += 360;


    // Serial.print("dt: "); Serial.print(dt);
    // Serial.print(" GxCal: "); Serial.print(GxCal);
    // Serial.print(" deltaRoll: "); Serial.println(deltaRoll);

    // Serial.print("yawC:");Serial.print(yawC);Serial.print(',');
    // Serial.print("yawM:");Serial.print(yawM);Serial.print(',');
    // Serial.print("yawG:");Serial.print(yawG);Serial.print(',');
    // Serial.print("LL:");Serial.print(-90);Serial.print(',');
    // Serial.print("UL:");Serial.println(90);

    Serial.print("rollC:");Serial.print(rollC);Serial.print(',');
    Serial.print("pitchC:");Serial.print(pitchC);Serial.print(',');
    Serial.print("yawC:");Serial.print(yawC);Serial.print(',');
    Serial.print("LL:");Serial.print(-90);Serial.print(',');
    Serial.print("UL:");Serial.println(90);
    // Serial.print(millis());Serial.print(',');
    // Serial.print(rollC);Serial.print(',');
    // Serial.print(pitchC);Serial.print(',');
    // Serial.println(yawC);

    delay(100);
}
#include "driver_MPU6050.h"


Driver_MPU6050::Driver_MPU6050(): mpu(Wire) {
    // Constructor can be empty or used for initialization if needed
}


bool Driver_MPU6050::init() {
    // Initialize the MPU6050 sensor
    byte status = mpu.begin();
    if(status != 0) {
        Serial.println("MPU6050 initialization failed!");
        while(1); // Initialization failed. Halt the program.
    }

    mpu.calcOffsets(); // Calibrate the sensor to find offsets


    // Warm up the filters by taking multiple readings and allowing sensor to stabilize
    Serial.println("Warming up filters...");
    for(int i=0; i<100; i++) {
        mpu.update();
        delay(10); 
    }

    baselineX = mpu.getAngleX();
    baselineZ = mpu.getAngleZ(); // Using Z-axis for roll since MPU6050's default orientation has roll around Z-axis

    Serial.print("Baseline X: ");
    Serial.println(baselineX);
    Serial.print("Baseline Z: ");
    Serial.println(baselineZ);

    return true; // Initialization successful
}

void Driver_MPU6050::update() {

    mpu.update();
    
    accelMag = sqrt(pow(mpu.getAccX(), 2) + pow(mpu.getAccY(), 2) + pow(mpu.getAccZ(), 2));

    raw_diff_x = mpu.getAngleX() - baselineX;
    // raw_diff_y = mpu.getAngleY() - baselineY; // not needed in this version but can be added later for more control
    raw_diff_z = mpu.getAngleZ() - baselineZ;


}

float Driver_MPU6050::getRawDiffX() const {
    return raw_diff_x;
}

float Driver_MPU6050::getRawDiffZ() const {
    return raw_diff_z;
}


float Driver_MPU6050::getAccelMag() const {
    return accelMag;
}

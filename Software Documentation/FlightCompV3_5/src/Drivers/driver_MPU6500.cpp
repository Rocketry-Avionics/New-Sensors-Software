#include "driver_MPU6500.h"

Driver_MPU6500::Driver_MPU6500(): mpu(SPI, PIN_CS_MPU) {
}

bool Driver_MPU6500::init() {
    byte status = mpu.begin();
    if (status != 0) {
        Serial.println("MPU6500 initialization failed!");
        while(1); // Halt
    }

    mpu.calcOffsets();

    Serial.println("Warming up filters...");
    for (int i = 0; i < 100; i++) {
        mpu.update();
        delay(10);
    }

    baselineX = mpu.getAngleX();
    baselineZ = mpu.getAngleZ();

    Serial.print("Baseline X: ");
    Serial.println(baselineX);
    Serial.print("Baseline Z: ");
    Serial.println(baselineZ);

    return true;
}

void Driver_MPU6500::update() {
    mpu.update();

    accelMag = sqrt(pow(mpu.getAccX(), 2) + pow(mpu.getAccY(), 2) + pow(mpu.getAccZ(), 2));

    raw_diff_x = mpu.getAngleX() - baselineX;
    raw_diff_z = mpu.getAngleZ() - baselineZ;
}

float Driver_MPU6500::getRawDiffX() const {
    return raw_diff_x;
}

float Driver_MPU6500::getRawDiffZ() const {
    return raw_diff_z;
}

float Driver_MPU6500::getAccelMag() const {
    return accelMag;
}

float Driver_MPU6500::getAccX() const {
    return mpu.getAccX();
}

float Driver_MPU6500::getAccY() const {
    return mpu.getAccY();
}

float Driver_MPU6500::getAccZ() const {
    return mpu.getAccZ();
}

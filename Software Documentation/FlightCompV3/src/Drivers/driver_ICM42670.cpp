#include "driver_ICM42670.h"
#include <math.h>

#define ACCEL_SCALE  2048.0f
#define GRAVITY      9.81f
#define GYRO_SCALE   16.384f

Driver_ICM42670::Driver_ICM42670() : icm(Wire, 0) {
}

void Driver_ICM42670::calcOffsets(int samples) {
    icm.startAccel(100, 16);
    icm.startGyro(100, 2000);

    int32_t sumAX = 0, sumAY = 0, sumAZ = 0;
    int32_t sumGX = 0, sumGY = 0, sumGZ = 0;

    inv_imu_sensor_event_t evt;

    for (int i = 0; i < samples; i++) {
        icm.getDataFromRegisters(evt);
        sumAX += evt.accel[0];
        sumAY += evt.accel[1];
        sumAZ += evt.accel[2];
        sumGX += evt.gyro[0];
        sumGY += evt.gyro[1];
        sumGZ += evt.gyro[2];
        delay(5);
    }

    accelOffsetX = sumAX / samples;
    accelOffsetY = sumAY / samples;
    accelOffsetZ = sumAZ / samples;
    gyroOffsetX  = sumGX / samples;
    gyroOffsetY  = sumGY / samples;
    gyroOffsetZ  = sumGZ / samples;
}

bool Driver_ICM42670::init() {
    int status = icm.begin();
    if (status != 0) {
        Serial.println("ICM42670P initialization failed!");
        while (1);
    }

    icm.startAccel(100, 16);
    icm.startGyro(100, 2000);

    calcOffsets(300); // sensor must be still during this

    // Use accelerometer ONLY HERE to get starting angle while rocket is on the pad
    Serial.println("Coicmting baseline angles from accelerometer...");
    inv_imu_sensor_event_t evt;
    float sumX = 0, sumZ = 0;
    for (int i = 0; i < 100; i++) {
        icm.getDataFromRegisters(evt);
        float ax = ((evt.accel[0] - accelOffsetX) / ACCEL_SCALE) * GRAVITY;
        float ay = ((evt.accel[1] - accelOffsetY) / ACCEL_SCALE) * GRAVITY;
        float az = ((evt.accel[2] - accelOffsetZ) / ACCEL_SCALE) * GRAVITY;
        sumX += atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;
        sumZ += atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
        delay(10);
    }

    // These are set once and never touched again after liftoff
    angleX    = sumX / 100.0f;
    angleZ    = sumZ / 100.0f;
    baselineX = angleX;
    baselineZ = angleZ;

    lastTime = millis();

    Serial.print("Baseline X: "); Serial.println(baselineX);
    Serial.print("Baseline Z: "); Serial.println(baselineZ);

    return true;
}

void Driver_ICM42670::update() {
    inv_imu_sensor_event_t evt;
    icm.getDataFromRegisters(evt);

    float ax = ((evt.accel[0] - accelOffsetX) / ACCEL_SCALE) * GRAVITY;
    float ay = ((evt.accel[1] - accelOffsetY) / ACCEL_SCALE) * GRAVITY;
    float az = ((evt.accel[2] - accelOffsetZ) / ACCEL_SCALE) * GRAVITY;
    float gx = (evt.gyro[0] - gyroOffsetX) / GYRO_SCALE;
    float gz = (evt.gyro[2] - gyroOffsetZ) / GYRO_SCALE;

    // dt
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    // Pure gyro integration — no accelerometer involved at all
    angleX += gx * dt;
    angleZ += gz * dt;

    // accel magnitude in m/s²
    accelMag = sqrt(ax * ax + ay * ay + az * az);

    raw_diff_x = angleX - baselineX;
    raw_diff_z = angleZ - baselineZ;
}

float Driver_ICM42670::getRawDiffX() const {
    return raw_diff_x;
}

float Driver_ICM42670::getRawDiffZ() const {
    return raw_diff_z;
}

float Driver_ICM42670::getAccelMag() const {
    return accelMag;
}
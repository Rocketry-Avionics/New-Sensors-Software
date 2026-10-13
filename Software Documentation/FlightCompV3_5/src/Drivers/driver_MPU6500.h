#ifndef DRIVER_MPU6500_H
#define DRIVER_MPU6500_H

#include "../libs/MPU6500_SPI.h"
#include "../Config/flightConfig.h"
#include <math.h>

class Driver_MPU6500 {
public:
    Driver_MPU6500();

    bool init();
    void update();

    float getRawDiffX() const;
    float getRawDiffZ() const;

    float getAccelMag() const;
    float getAccX() const;
    float getAccY() const;
    float getAccZ() const; // Axial / vertical acceleration in g

    float getGyroX() const;
    float getGyroY() const;
    float getGyroZ() const;

private:
    MPU6500_SPI mpu;

    double baselineX, baselineZ;
    double raw_diff_x, raw_diff_z;

    double accelMag;
};

#endif // DRIVER_MPU6500_H

#ifndef DRIVER_ICM42670P_H
#define DRIVER_ICM42670P_H

#include <Wire.h>
#include <ICM42670P.h>


#include "../Config/flightConfig.h"
#include <math.h>


class Driver_ICM42670 {
    public:
        
        Driver_ICM42670();
    
        void calcOffsets(int);
        bool init();
        
        void update();    

        float getRawDiffX() const;
        float getRawDiffZ() const;

        float getAccelMag() const;
    
    private:

        ICM42670 icm;
        unsigned long lastTime;
        float angleX, angleZ;
        double baselineX, baselineZ; //, baselineY; // Not used in this version but can be added later for more control
        double raw_diff_x, raw_diff_z; //, raw_diff_y; // Not used in this version but can be added later for more control

        double accelMag;
        
        int32_t accelOffsetX, accelOffsetY, accelOffsetZ; //for calibrating IMU
        int32_t gyroOffsetX,  gyroOffsetY,  gyroOffsetZ;

};


#endif
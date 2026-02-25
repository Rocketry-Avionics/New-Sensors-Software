#ifndef DRIVER_MPU6050_H
#define DRIVER_MPU6050_H

#include "../libs/MPU6050_light.h"
#include "../Config/flightConfig.h"
#include <math.h>


class Driver_MPU6050 {
    public:
        
        Driver_MPU6050();
    
        bool init();
        
        void update();    

        float getRawDiffX() const;
        float getRawDiffZ() const;

        float getAccelMag() const;
    
    private:

        MPU6050 mpu;
    
        double baselineX, baselineZ; //, baselineY; // Not used in this version but can be added later for more control
        double raw_diff_x, raw_diff_z; //, raw_diff_y; // Not used in this version but can be added later for more control

        double accelMag;

};


#endif
#ifndef DRIVER_BMP280_H
#define DRIVER_BMP280_H

#include <Adafruit_BMP280.h>
#include "../Config/flightConfig.h"

class Driver_BMP280 {
    public:
    
        Driver_BMP280();

        bool init();
        void update();

        float getAlt() const;
        float getTemp() const;
        float getPres() const;
    

    private:

        Adafruit_BMP280 bmp;

        float grndRefPres;

        float currentAlt;
        float currentTemp;
        float currentPres;

};

#endif
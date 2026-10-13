#include "driver_BMP280.h"

Driver_BMP280::Driver_BMP280(): bmp(PIN_CS_BMP, &SPI) {
    grndRefPres = 1013.25f;
    currentAlt = 0.0f;
    currentTemp = 0.0f;
    currentPres = 0.0f;
}

bool Driver_BMP280::init() {
    if (!bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor over SPI, check wiring!"));
        while (1); // Halt if sensor not found
    }

    /* Configure sensor mode, oversampling, filter, and standby duration for rocketry */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1);   /* Standby time (~1ms). */

    // Calculate ground reference pressure by averaging multiple readings
    grndRefPres = 0.0f;
    int successCount = 0;
    for (int i = 0; i < 10; i++) {
        float pres_hPa = bmp.readPressure() / 100.0f;
        if (pres_hPa > 300.0f && pres_hPa < 1200.0f) {
            grndRefPres += pres_hPa;
            successCount++;
        }
        delay(10);
    }

    if (successCount > 0) {
        grndRefPres /= static_cast<float>(successCount);
    } else {
        grndRefPres = 1013.25f;
    }

    Serial.print("Ground Pressure: ");
    Serial.print(grndRefPres);
    Serial.println(" hPa");

    return true;
}

void Driver_BMP280::update() {
    currentTemp = bmp.readTemperature();
    currentPres = bmp.readPressure() / 100.0f; // Pa to hPa
    currentAlt = bmp.readAltitude(grndRefPres);
}

float Driver_BMP280::getAlt() const {
    return currentAlt;
}

float Driver_BMP280::getTemp() const {
    return currentTemp;
}

float Driver_BMP280::getPres() const {
    return currentPres;
}

float Driver_BMP280::getGroundRefPres() const {
    return grndRefPres;
}

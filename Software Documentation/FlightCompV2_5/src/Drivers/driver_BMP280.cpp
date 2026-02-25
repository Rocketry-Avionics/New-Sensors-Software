#include "driver_BMP280.h"


Driver_BMP280::Driver_BMP280() : bmp() {
  // Constructor can be empty or used for initialization if needed
}


// Initialize the BMP280 sensor
bool Driver_BMP280::init() {


    if (!bmp.begin(0x76)) {        // Try the default I2C address
        if (!bmp.begin(0x77)) {      // Try the alternate I2C address
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));

            while (1); // Halt if no sensor is found
        
        }
    }
    
    // Configure the sensor (you can adjust these settings as needed)
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // Operating Mode 
                    Adafruit_BMP280::SAMPLING_X1,   // Temp. oversampling 
                    Adafruit_BMP280::SAMPLING_X4,   // Pressure oversampling
                    Adafruit_BMP280::FILTER_X4,    // Filtering
                    Adafruit_BMP280::STANDBY_MS_1); // Standby time
        
    // Calculate ground reference pressure by averaging multiple readings
    grndRefPres = 0;
    for(int i=0; i<10; i++) {
        grndRefPres += bmp.readPressure();
        delay(10);
    }

    grndRefPres = (grndRefPres / 10.0) / 100.0; // Average and convert to hPa
    Serial.print("Ground Pressure: ");
    Serial.print(grndRefPres);
    Serial.println(" hPa");

    return true; // Initialization successful
}

void Driver_BMP280::update() {
    // Read the latest sensor data and store it in the member variables
    currentTemp = bmp.readTemperature();
    currentPres = bmp.readPressure();
    currentAlt = bmp.readAltitude(grndRefPres); // Use ground reference pressure for altitude calculation
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


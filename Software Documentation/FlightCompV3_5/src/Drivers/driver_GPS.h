#ifndef DRIVER_GPS_H
#define DRIVER_GPS_H

#include <Arduino.h>
#include <TinyGPS++.h>

class Driver_GPS {
public:
    Driver_GPS();

    // Call once in setup(). Starts Serial1, then sends UBX commands
    // to upgrade the NEO-M8N to 115200 baud and 10 Hz update rate.
    void init(uint32_t initialBaudRate = 9600);

    // Call every iteration of loop(). Non-blocking: drains Serial1
    // into TinyGPS++ parser. Does NOT block waiting for new data.
    void update();

    // Returns true if a valid GPS fix has been received in the last 2 seconds.
    bool isFixed();

    // Returns latitude in decimal degrees. Returns 0.0 if no fix.
    double getLat();

    // Returns longitude in decimal degrees. Returns 0.0 if no fix.
    double getLng();

    // Returns GPS altitude in metres above mean sea level. Returns 0.0 if no fix.
    double getGPSAlt();

    // Returns speed over ground in km/h. Returns 0.0 if no fix.
    double getSpeed();

    // Returns number of satellites tracked. Returns 0 if no fix.
    uint32_t getSatellites();

private:
    TinyGPSPlus gps;

    // Writes a raw byte array to Serial1 (used to send UBX binary commands).
    void sendUBX(const uint8_t* cmd, uint16_t len);

    // Sends UBX commands to set 115200 baud and 10 Hz measurement rate.
    void configureHighRate();
};

#endif // DRIVER_GPS_H

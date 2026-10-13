#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <Arduino.h>
#include <SDMMCBlockDevice.h>
#include <FATFileSystem.h>
#include <stdio.h>
#include "../Config/flightConfig.h"

class Blackbox {
public:
    Blackbox();

    // Call once in setup() after baro.init() to supply ground baseline pressure.
    // Mounts SDMMC filesystem and creates the next sequential flight_XXX.csv.
    // Returns true on success; gracefully disables logging on failure without halting.
    bool init(float groundRefPres);

    // Call every loop(). Rate-limited internally to 50 Hz (LOG_INTERVAL_MS = 20ms).
    // Writes strictly 17 raw primary channels to the active log stream.
    void logRecord(unsigned long time_ms,
                   uint8_t state,
                   float pres_hpa,
                   float temp_c,
                   float ax_g,
                   float ay_g,
                   float az_g,
                   float gx_dps,
                   float gy_dps,
                   float gz_dps,
                   double servo_x,
                   double servo_y,
                   bool chute,
                   double lat,
                   double lng,
                   double gps_alt_m,
                   uint32_t sats);

    // Flushes buffered data to physical MicroSD flash.
    // Called periodically (1 Hz) and immediately on flight state transitions.
    void flush();

    // Returns true if SD card mounted and file successfully opened.
    bool isReady() const { return _isReady; }

    // Returns the current log file path (e.g. "/fs/flight_001.csv").
    const char* getFilePath() const { return _filePath; }

private:
    SDMMCBlockDevice _blockDevice;
    mbed::FATFileSystem _fs;
    FILE* _file;

    bool _isReady;
    unsigned long _lastLogMs;
    unsigned long _lastFlushMs;
    char _filePath[32];
};

#endif // BLACKBOX_H

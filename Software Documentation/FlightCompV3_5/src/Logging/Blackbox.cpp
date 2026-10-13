#include "Blackbox.h"

Blackbox::Blackbox()
    : _blockDevice(),
      _fs("fs"),
      _file(NULL),
      _isReady(false),
      _lastLogMs(0),
      _lastFlushMs(0) {
    _filePath[0] = '\0';
}

bool Blackbox::init(float groundRefPres) {
    // Attempt to mount the SDMMC file system on the Portenta Breakout board
    int err = _fs.mount(&_blockDevice);
    if (err) {
        Serial.print("Blackbox: Failed to mount SD card filesystem (code: ");
        Serial.print(err);
        Serial.println(") — flight continuing without logging.");
        _isReady = false;
        return false;
    }

    // Auto-increment filename: find next available flight_XXX.csv
    int flightIndex = 1;
    char testPath[32];
    for (int i = 1; i <= 999; i++) {
        snprintf(testPath, sizeof(testPath), "/fs/flight_%03d.csv", i);
        FILE* testFile = fopen(testPath, "r");
        if (testFile) {
            fclose(testFile);
        } else {
            flightIndex = i;
            break;
        }
    }

    snprintf(_filePath, sizeof(_filePath), "/fs/flight_%03d.csv", flightIndex);

    _file = fopen(_filePath, "w");
    if (!_file) {
        Serial.println("Blackbox: Failed to create log file on SD card!");
        _isReady = false;
        return false;
    }

    // Write header metadata and column headers
    fprintf(_file, "# FlightCompV3_5 Blackbox Raw Flight Log\n");
    fprintf(_file, "# ground_ref_pres_hpa: %.2f\n", groundRefPres);
    fprintf(_file, "time_ms,state,pres_hpa,temp_c,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,servo_x,servo_y,chute,lat,lng,gps_alt_m,sats\n");
    fflush(_file);

    _isReady = true;
    _lastFlushMs = millis();
    Serial.print("Blackbox: Logging initialized -> ");
    Serial.println(_filePath);
    return true;
}

void Blackbox::logRecord(unsigned long time_ms,
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
                         uint32_t sats) {
    if (!_isReady || !_file) return;

    unsigned long now = millis();
    if (now - _lastLogMs < LOG_INTERVAL_MS) return;
    _lastLogMs = now;

    fprintf(_file, "%lu,%u,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%.6f,%.6f,%.1f,%lu\n",
        time_ms,
        (unsigned int)state,
        pres_hpa,
        temp_c,
        ax_g,
        ay_g,
        az_g,
        gx_dps,
        gy_dps,
        gz_dps,
        servo_x,
        servo_y,
        chute ? 1 : 0,
        lat,
        lng,
        gps_alt_m,
        (unsigned long)sats
    );

    // Periodic flush to prevent data loss on unexpected power loss
    if (now - _lastFlushMs >= LOG_FLUSH_INTERVAL_MS) {
        flush();
    }
}

void Blackbox::flush() {
    if (!_isReady || !_file) return;
    fflush(_file);
    _lastFlushMs = millis();
}

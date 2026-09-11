#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <LoRa.h>
#include "../Config/flightConfig.h"

class Telemetry {
public:
    Telemetry();

    // Call once in setup() after SPI.begin() and after LoRa CS pin is driven HIGH.
    // Configures and starts the SX1278 LoRa radio. Returns true on success.
    bool init();

    // Call every loop(). Internally rate-limited to once per 500ms.
    // Builds and transmits a CSV telemetry packet over LoRa.
    // Packet format: "alt,maxAlt,accelMag,lat,lng,chute,state"
    //   alt         — barometric altitude in metres (1 decimal place)
    //   maxAlt      — maximum altitude recorded in metres (1 decimal place)
    //   accelMag    — acceleration magnitude in g (2 decimal places)
    //   lat         — GPS latitude in decimal degrees (6 decimal places)
    //   lng         — GPS longitude in decimal degrees (6 decimal places)
    //   chute       — parachute deployed flag: "1" = deployed, "0" = not deployed
    //   state       — flight state (0=PAD, 1=BOOST, 2=COAST, 3=APOGEE, 4=CHUTE)
    void transmitData(float altitude,
                      float maxAltitude,
                      float accelMag,
                      double lat,
                      double lng,
                      bool parachuteDeployed,
                      uint8_t flightState = 0);

private:
    bool _initialized;
    unsigned long _lastTxMs;

    // Minimum time between transmissions in milliseconds.
    // LoRa TX can take ~20ms at SF9/125kHz; rate-limiting prevents
    // the control loop from being stalled on every iteration.
    static const unsigned long TX_INTERVAL_MS = 500UL;
};

#endif // TELEMETRY_H

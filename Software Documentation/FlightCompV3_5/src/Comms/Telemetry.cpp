#include "Telemetry.h"

Telemetry::Telemetry()
    : _initialized(false), _lastTxMs(0) {
}

bool Telemetry::init() {
    // Configure LoRa pins BEFORE calling LoRa.begin().
    // Order: setPins → setSPIFrequency → begin.
    // NSS=PIN_CS_LORA (D4), RST=PIN_LORA_RST (D2), DIO0=PIN_LORA_DIO0 (D3)
    LoRa.setPins(PIN_CS_LORA, PIN_LORA_RST, PIN_LORA_DIO0);

    // Limit SPI clock to 1 MHz for stability on the Portenta H7 Mbed OS core.
    // The SX1278 supports up to 10 MHz, so 1 MHz is well within spec.
    LoRa.setSPIFrequency(1000000);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa: SX1278 init FAILED — check wiring and CS pin");
        _initialized = false;
        return false;
    }

    // RF configuration for model rocketry telemetry.
    // SF9 / 125kHz / CR4/5 gives a good balance of range and air-time.
    // At SF9 with a 20-byte packet: ~150ms air time, ~3-5km range.
    LoRa.setSpreadingFactor(9);       // SF9: 2^9 chips/symbol
    LoRa.setSignalBandwidth(125E3);   // 125 kHz (standard LoRa bandwidth)
    LoRa.setCodingRate4(5);           // 4/5 coding rate (lowest overhead)

    _initialized = true;
    Serial.println("LoRa: SX1278 initialized at 433 MHz");
    return true;
}

void Telemetry::transmitData(float altitude,
                              float maxAltitude,
                              float accelMag,
                              double lat,
                              double lng,
                              bool parachuteDeployed,
                              uint8_t flightState) {
    // Do nothing if init() failed.
    if (!_initialized) return;

    // Rate-limit: skip this call if fewer than TX_INTERVAL_MS have elapsed
    // since the last successful transmission.
    unsigned long now = millis();
    if (now - _lastTxMs < TX_INTERVAL_MS) return;
    _lastTxMs = now;

    // Build and transmit the CSV packet.
    // LoRa.endPacket(true) = async TX — uses DIO0 interrupt for completion;
    // does not block the CPU while the radio transmits.
    LoRa.beginPacket();
    LoRa.print(altitude, 1);                        // e.g. "123.4"
    LoRa.print(",");
    LoRa.print(maxAltitude, 1);                     // e.g. "145.2"
    LoRa.print(",");
    LoRa.print(accelMag, 2);                        // e.g. "9.81"
    LoRa.print(",");
    LoRa.print(lat, 6);                             // e.g. "12.971598"
    LoRa.print(",");
    LoRa.print(lng, 6);                             // e.g. "77.594562"
    LoRa.print(",");
    LoRa.print(parachuteDeployed ? "1" : "0");      // "1" or "0"
    LoRa.print(",");
    LoRa.print(flightState);                        // e.g. "0", "1", "2", "3", "4"
    LoRa.endPacket(true);                           // async (non-blocking) TX
}

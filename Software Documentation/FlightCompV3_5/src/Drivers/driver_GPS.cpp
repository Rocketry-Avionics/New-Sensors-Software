#include "driver_GPS.h"

// ─── UBX Binary Commands ───────────────────────────────────────────────────
//
// These are pre-computed UBX protocol packets for the u-blox NEO-M8N.
// They configure UART1 baud rate and measurement rate without needing
// u-center or any external tools. The module applies them immediately.
//
// If the module has onboard flash, it retains these settings across power
// cycles. If not, they are re-applied every boot — this is safe and correct.

// UBX-CFG-PRT: Set UART1 baud rate to 115200
// (Required before 10 Hz — at 9600 baud, 10 Hz NMEA overflows the buffer)
static const uint8_t UBX_SET_BAUD_115200[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x00,             // Class=CFG, ID=PRT
    0x14, 0x00,             // Payload length = 20 bytes
    0x01,                   // portID = 1 (UART1)
    0x00,                   // reserved0
    0x00, 0x00,             // txReady (disabled)
    0xD0, 0x08, 0x00, 0x00, // mode: 8N1
    0x00, 0xC2, 0x01, 0x00, // baudRate = 115200 (0x0001C200)
    0x23, 0x00,             // inProtoMask: UBX+NMEA
    0x23, 0x00,             // outProtoMask: UBX+NMEA
    0x00, 0x00,             // flags
    0x00, 0x00,             // reserved1
    0xDE, 0xC9              // checksum (CK_A, CK_B)
};

// UBX-CFG-RATE: Set measurement period to 100ms (10 Hz), navigation rate = 1
static const uint8_t UBX_SET_10HZ[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x08,             // Class=CFG, ID=RATE
    0x06, 0x00,             // Payload length = 6 bytes
    0x64, 0x00,             // measRate = 100ms (0x0064) → 10 Hz
    0x01, 0x00,             // navRate = 1 (1 navigation solution per measurement)
    0x01, 0x00,             // timeRef = 1 (GPS time)
    0x7A, 0x12              // checksum (CK_A, CK_B)
};

// ─── Implementation ────────────────────────────────────────────────────────

Driver_GPS::Driver_GPS() {
    // TinyGPSPlus default-constructs safely. Nothing else needed here.
}

void Driver_GPS::sendUBX(const uint8_t* cmd, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        Serial1.write(cmd[i]);
    }
    Serial1.flush(); // Ensure all bytes are transmitted before continuing
}

void Driver_GPS::configureHighRate() {
    // Step 1: Send the baud-rate change command while still at 9600 baud.
    // The module will switch baud rate immediately after receiving this packet.
    sendUBX(UBX_SET_BAUD_115200, sizeof(UBX_SET_BAUD_115200));
    delay(100); // Wait for module to switch baud rate

    // Step 2: Close Serial1 and reopen at the new baud rate (115200).
    Serial1.end();
    delay(50);
    Serial1.begin(115200);
    delay(100); // Wait for Serial1 hardware to stabilise

    // Step 3: Send the 10 Hz rate configuration at the new baud rate.
    sendUBX(UBX_SET_10HZ, sizeof(UBX_SET_10HZ));
    delay(100); // Wait for module to apply the rate change
}

void Driver_GPS::init(uint32_t initialBaudRate) {
    // Open Serial1 at the module's factory default baud rate (9600).
    // Serial1 maps to D13 (RX) and D14 (TX) on the Portenta H7.
    Serial1.begin(initialBaudRate);
    delay(500); // Allow the module's UART to stabilise after power-up

    // Upgrade to 115200 baud and 10 Hz NMEA output.
    configureHighRate();

    Serial.println("GPS: NEO-M8N initialized (115200 baud, 10 Hz)");
}

void Driver_GPS::update() {
    // Drain all available bytes from the GPS serial port into the parser.
    // TinyGPS++ updates its internal state incrementally with each byte.
    // This is non-blocking — returns immediately if no data is available.
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }
}

bool Driver_GPS::isFixed() {
    // isValid() = parser has received a valid location sentence
    // age() < 2000 = last fix was received within the past 2 seconds
    return gps.location.isValid() && (gps.location.age() < 2000);
}

double Driver_GPS::getLat() {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
}

double Driver_GPS::getLng() {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
}

double Driver_GPS::getGPSAlt() {
    return gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
}

double Driver_GPS::getSpeed() {
    return gps.speed.isValid() ? gps.speed.kmph() : 0.0;
}

uint32_t Driver_GPS::getSatellites() {
    return gps.satellites.isValid() ? gps.satellites.value() : 0U;
}

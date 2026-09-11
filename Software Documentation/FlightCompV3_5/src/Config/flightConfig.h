#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

// Board Selection
// #define HW_TARGET_TEENSY_41
#define HW_TARGET_PORTENTA_H7

// Pin Definitions
#ifdef HW_TARGET_TEENSY_41
    const int PIN_SERVO_X = 24;
    const int PIN_SERVO_Y = 25;
    const int PIN_CHUTE_SERVO = 26;

#elif defined(HW_TARGET_PORTENTA_H7)
    // SPI Chip Select Pins — match to schematic
    const int PIN_CS_MPU = 7;        // Portenta D7 — MPU6500 NCS
    const int PIN_CS_BMP = 5;        // Portenta D5 — BMP280 CSB

    // LoRa SX1278 Pins — match to schematic
    const int PIN_CS_LORA   = 4;     // Portenta D4 — LoRa NSS/CS
    const int PIN_LORA_DIO0 = 3;     // Portenta D3 — LoRa DIO0 (TX-done IRQ)
    const int PIN_LORA_RST  = 2;     // Portenta D2 — LoRa RST

    // Servo PWM Output Pins — match to schematic
    const int PIN_SERVO_X     = 0;   // Portenta D0
    const int PIN_SERVO_Y     = 1;   // Portenta D1
    const int PIN_CHUTE_SERVO = 11;  // Portenta D11 (relocated from D2 to free pin)
#endif


const double DEADBAND = 1.0; // Degrees of tolerance before PID kicks in

// Parachute Deployment Thresholds and Constraints (all in metres)
const float APOGEE_DROP_THRES = 2.0;
const float MIN_ARM_ALTITUDE  = 60.0;

// Recovery Backup Failsafe Timer (in milliseconds)
// Maximum elapsed time allowed in COAST post-burnout before forcing APOGEE detection
const unsigned long APOGEE_BACKUP_TIMEOUT_MS = 10000UL; // 10 seconds post-burnout

// State Machine Acceleration & Altitude Thresholds
const float LIFTOFF_ACCEL_THRES = 2.5f; // Z-acceleration in g to trigger BOOST
const float LIFTOFF_ALT_THRES   = 5.0f; // Altitude above pad in m to confirm BOOST
const float BURNOUT_ACCEL_THRES = 0.0f; // Z-acceleration in g below which motor has burned out (sign flip!)

// State Machine Debounce Sample Counts (@ 250 Hz loop)
const int DEBOUNCE_LIFTOFF_COUNT = 3; // ~12 ms
const int DEBOUNCE_BURNOUT_COUNT = 4; // ~16 ms
const int DEBOUNCE_APOGEE_COUNT  = 3; // ~12 ms

// Parachute Canopy Inflation Verification (Longitudinal Z-axis only)
const float CHUTE_STEADY_AZ_MIN = 0.85f; // Minimum az in g for steady canopy descent
const float CHUTE_STEADY_AZ_MAX = 1.15f; // Maximum az in g for steady canopy descent
const unsigned long CHUTE_CONFIRM_DURATION_MS = 3000UL; // 3.0 seconds of steady ~1g az to confirm canopy

// Serial Debug Output Rate Limit
const unsigned long SERIAL_DEBUG_INTERVAL_MS = 100UL; // 10 Hz rate limit for USB/UART debug stream

// PID Gains
const double PID_KP = 2.0;
const double PID_KI = 0.0;
const double PID_KD = 2.5;

// PID Calculation Period (in milliseconds)
// 4ms period corresponds to ~250 Hz calculation rate matching the MPU6500 update loop
const int PID_SAMPLE_TIME_MS = 4;

// Complementary Filter Weight
const float FILTER_ALPHA = 0.95;

// Kalman Filter Constants
const double KALMAN_R = 40.0;
const double KALMAN_H = 1.0;
const double KALMAN_Q = 10.0;

// LoRa Radio Frequency — 433 MHz ISM band (India), exact 32-bit integer
const long LORA_FREQUENCY = 433000000L;

#endif
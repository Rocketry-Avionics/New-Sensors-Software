#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

#include "Config/flightConfig.h"
#include "Drivers/driver_MPU6500.h"
#include "Drivers/driver_BMP280.h"
#include "Drivers/driver_GPS.h"
#include "Control/flightController.h"
#include "Comms/Telemetry.h"
#include "Logging/Blackbox.h"


// Flight State Machine
enum FlightState {
    STATE_PAD    = 0,
    STATE_BOOST  = 1,
    STATE_COAST  = 2,
    STATE_APOGEE = 3,
    STATE_CHUTE  = 4
};

FlightState flightState = STATE_PAD;

Driver_MPU6500 imu;
Driver_BMP280 baro;
Driver_GPS gps;

FlightController controller;
Servo parachuteServo;

Telemetry radio;
Blackbox blackbox;

double maxAltitude = 0.0;
double altitude = 0.0;
bool parachuteDeployed = false;

// State Machine Debounce & Timing Trackers
int liftoffDebounceCount = 0;
int burnoutDebounceCount = 0;
int apogeeDebounceCount  = 0;
unsigned long burnoutTimestampMs = 0;
unsigned long chuteSteadyStartMs = 0;

// Loop Rate Limiting & Diagnostics
unsigned long lastDebugPrintMs = 0;
unsigned long lastHeartbeatMs = 0;
bool heartbeatLedState = false;


void setup() {
    Serial.begin(9600);

    // Drive all SPI chip select pins HIGH BEFORE calling SPI.begin().
    // This prevents any device from being accidentally selected during
    // SPI bus initialisation, which could corrupt bus state.
    pinMode(PIN_CS_MPU,  OUTPUT);
    digitalWrite(PIN_CS_MPU,  HIGH);
    pinMode(PIN_CS_BMP,  OUTPUT);
    digitalWrite(PIN_CS_BMP,  HIGH);
    pinMode(PIN_CS_LORA, OUTPUT);
    digitalWrite(PIN_CS_LORA, HIGH);

    SPI.begin();
    delay(2000);
    Serial.println("Flight Computer V3.5 Booting...");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // Active-low on Portenta H7: LOW turns LED on

    baro.init();
    baro.update(); // Perform initial reading to initialize internal altitude state
    maxAltitude = baro.getAlt(); // Set initial max altitude to current altitude for accurate apogee detection

    imu.init();
    controller.init();

    // Setup the parachute servo
    parachuteServo.attach(PIN_CHUTE_SERVO);
    parachuteServo.write(90); // Locked position

    // Initialize GPS on Serial1 (D13=RX, D14=TX).
    // Sends UBX commands to configure 115200 baud and 10 Hz update rate.
    gps.init();

    // Initialize LoRa SX1278 on SPI1.
    // Pins: NSS=D4, RST=D2, DIO0=D3. Frequency: 433 MHz.
    radio.init();

    // Initialize Blackbox SDMMC high-rate data logger on Portenta Breakout board
    blackbox.init(baro.getGroundRefPres());

    Serial.println("Flight Computer V3.5 Ready. State: STATE_PAD");
}

void loop() {
    imu.update();
    baro.update();
    gps.update(); // Non-blocking: drains Serial1 bytes into TinyGPS++ parser

    altitude = baro.getAlt();
    float accelMag = imu.getAccelMag();
    float az = imu.getAccZ(); // Vertical acceleration in g

    // Track Peak Altitude
    if (altitude > maxAltitude) {
        maxAltitude = altitude;
    }

    // ─── Flight State Machine ──────────────────────────────────────────────
    switch (flightState) {
        case STATE_PAD:
            // Dual confirmation: high axial acceleration (> 2.5g) AND altitude above pad (> 5m)
            // debounced across consecutive samples to reject pad handling bumps.
            if (az > LIFTOFF_ACCEL_THRES && altitude > LIFTOFF_ALT_THRES) {
                liftoffDebounceCount++;
                if (liftoffDebounceCount >= DEBOUNCE_LIFTOFF_COUNT) {
                    flightState = STATE_BOOST;
                    liftoffDebounceCount = 0;
                    blackbox.flush();
                    Serial.println("\n>>> LIFTOFF DETECTED - STATE_BOOST <<<");
                }
            } else {
                liftoffDebounceCount = 0;
            }
            break;

        case STATE_BOOST:
            // Burnout is detected when axial acceleration flips negative (az <= 0.0g)
            // due to thrust cutoff and aerodynamic drag, debounced across samples.
            if (az <= BURNOUT_ACCEL_THRES) {
                burnoutDebounceCount++;
                if (burnoutDebounceCount >= DEBOUNCE_BURNOUT_COUNT) {
                    flightState = STATE_COAST;
                    burnoutDebounceCount = 0;
                    burnoutTimestampMs = millis();
                    blackbox.flush();
                    Serial.println("\n>>> BURNOUT DETECTED - STATE_COAST <<<");
                }
            } else {
                burnoutDebounceCount = 0;
            }
            break;

        case STATE_COAST: {
            bool apogeeConfirmed = false;

            // Primary check: confirm decreasing altitude after reaching arming altitude
            if (maxAltitude > MIN_ARM_ALTITUDE && (maxAltitude - altitude) >= APOGEE_DROP_THRES) {
                apogeeDebounceCount++;
                if (apogeeDebounceCount >= DEBOUNCE_APOGEE_COUNT) {
                    apogeeConfirmed = true;
                    Serial.println("\n>>> APOGEE DETECTED (Baro Drop) - STATE_APOGEE <<<");
                }
            } else {
                apogeeDebounceCount = 0;
            }

            // Backup Safety Failsafe: Trigger if timeout expires post-burnout
            if (millis() - burnoutTimestampMs >= APOGEE_BACKUP_TIMEOUT_MS) {
                apogeeConfirmed = true;
                Serial.println("\n>>> Backup Failsafe Timer Triggered - STATE_APOGEE <<<");
            }

            if (apogeeConfirmed) {
                flightState = STATE_APOGEE;
                apogeeDebounceCount = 0;

                // Immediately actuate parachute servo upon entering STATE_APOGEE
                parachuteServo.write(180);
                parachuteDeployed = true;
                chuteSteadyStartMs = 0;
                blackbox.flush();
                Serial.println("\n!!! APOGEE DETECTED - PARACHUTE SERVO ACTUATED !!!");
            }
            break;
        }

        case STATE_APOGEE:
            // Parachute servo has been commanded. Verify canopy inflation:
            // When descending steadily under canopy, longitudinal acceleration az
            // sits consistently near 1.0g (drag balancing rocket weight).
            // Lateral acceleration is excluded to prevent wind gusts from disrupting verification.
            if (az >= CHUTE_STEADY_AZ_MIN && az <= CHUTE_STEADY_AZ_MAX) {
                if (chuteSteadyStartMs == 0) {
                    chuteSteadyStartMs = millis();
                } else if (millis() - chuteSteadyStartMs >= CHUTE_CONFIRM_DURATION_MS) {
                    flightState = STATE_CHUTE;
                    blackbox.flush();
                    Serial.println("\n>>> PARACHUTE CANOPY CONFIRMED - STATE_CHUTE <<<");
                }
            } else {
                // Reset timer if az fluctuates outside steady descent window
                chuteSteadyStartMs = 0;
            }
            break;

        case STATE_CHUTE:
            // Parachute deployed, rocket descending under canopy
            break;
    }

    // Active Stabilization PID Control
    controller.update(imu.getRawDiffX(), imu.getRawDiffZ());

    unsigned long now = millis();

    // ─── High-Rate Blackbox Logging (50 Hz) ────────────────────────────────
    blackbox.logRecord(
        now,
        (uint8_t)flightState,
        baro.getPres(),
        baro.getTemp(),
        imu.getAccX(),
        imu.getAccY(),
        imu.getAccZ(),
        imu.getGyroX(),
        imu.getGyroY(),
        imu.getGyroZ(),
        controller.getServoX(),
        controller.getServoY(),
        parachuteDeployed,
        gps.getLat(),
        gps.getLng(),
        gps.getGPSAlt(),
        gps.getSatellites()
    );

    // ─── Rate-Limited Debug Output (10 Hz) ──────────────────────────────────
    if (now - lastDebugPrintMs >= SERIAL_DEBUG_INTERVAL_MS) {
        lastDebugPrintMs = now;
        Serial.print("\nAlt: ");
        Serial.print(altitude, 1);
        Serial.print(" m | Max: ");
        Serial.print(maxAltitude, 1);
        Serial.print(" m | az: ");
        Serial.print(az, 2);
        Serial.print(" g | State: ");
        switch (flightState) {
            case STATE_PAD:    Serial.print("PAD");    break;
            case STATE_BOOST:  Serial.print("BOOST");  break;
            case STATE_COAST:  Serial.print("COAST");  break;
            case STATE_APOGEE: Serial.print("APOGEE"); break;
            case STATE_CHUTE:  Serial.print("CHUTE");  break;
        }
        Serial.print(" | Chute: ");
        Serial.print(parachuteDeployed ? "1" : "0");
    }

    // ─── Heartbeat LED Toggle (1 Hz) ───────────────────────────────────────
    if (now - lastHeartbeatMs >= 500UL) {
        lastHeartbeatMs = now;
        heartbeatLedState = !heartbeatLedState;
        digitalWrite(LED_BUILTIN, heartbeatLedState ? LOW : HIGH); // Active-low
    }

    // ─── LoRa Telemetry Broadcast (2 Hz) ───────────────────────────────────
    // Packet format: "alt,maxAlt,accelMag,lat,lng,chute,state"
    radio.transmitData(
        (float)altitude,
        (float)maxAltitude,
        accelMag,
        gps.getLat(),
        gps.getLng(),
        parachuteDeployed,
        (uint8_t)flightState
    );
}


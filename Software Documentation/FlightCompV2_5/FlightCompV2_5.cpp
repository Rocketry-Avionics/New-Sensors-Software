#include <Arduino.h>
#include <Servo.h>

#include "src/Config/FlightConfig.h"
#include "src/Drivers/Driver_MPU6050.h"
#include "src/Drivers/Driver_BMP280.h"
#include "src/Control/FlightController.h"
// #include "src/Comms/Telemetry.h"


Driver_MPU6050 imu;
Driver_BMP280 baro;

FlightController controller;
Servo parachuteServo;

// Telemetry radio;

double maxAltitude = 0.0;
double altitude = 0.0;
bool parachuteDeployed = false;


void setup() {
    Serial.begin(9600);
    delay(2000);
    Serial.println("Flight Computer Booting...");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED to indicate that Teensy is alive
    
    baro.init();
    maxAltitude = baro.getAlt(); // Set initial max altitude to current altitude for accurate apogee detection

    imu.init();
    controller.init(); 

    // Setup the parachute
    parachuteServo.attach(PIN_CHUTE_SERVO);
    parachuteServo.write(90); // Locked position
    
    // radio.init();

}

void loop() {

    imu.update();
    baro.update();
    
    altitude = baro.getAlt();
    double accelMag = imu.getAccelMag();

    // Track Max Altitude
    if (altitude > maxAltitude) {
        maxAltitude = altitude;
    }

    // A. Apogee Detection Logic
    if (!parachuteDeployed && maxAltitude > MIN_ARM_ALTITUDE) {
        if (accelMag < 0.5) { // Must be a different value. Change it later.
            if ((maxAltitude - altitude) >= APOGEE_DROP_THRES) {
                parachuteServo.write(180); // Deploy parachute

                parachuteDeployed = true;

                Serial.println("\n!!! APOGEE DETECTED - PARACHUTE DEPLOYED !!!");
            }
        }
    }


    controller.update(imu.getRawDiffX(), imu.getRawDiffZ()); // Y is optional for now, can be added in future versions

    Serial.printf("\nAltitude: %f m", altitude);
    Serial.printf(" | Max Altitude: %f m", maxAltitude);
    Serial.printf(" | Accel Mag: %f m/s^2", accelMag);
    
    // F. Transmit Data to Ground Station
    // radio.transmitData(altitude, controller.getPitch(), controller.getRoll());
}
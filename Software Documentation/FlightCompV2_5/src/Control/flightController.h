#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H


#include <Arduino.h>
#include <Servo.h>
#include "../libs/PID_v1.h" // Assuming this is in your project root or libraries folder
#include "../Config/flightConfig.h"
#include "kalman.h"
#include <math.h>


class FlightController {
    public:
        FlightController();

        void init();

        void update(double raw_diff_x, double raw_diff_z, double raw_diff_y = 0.0); // Y is optional for now, can be used in future versions

        void printDebug() const; // Optional: For printing internal states for debugging

    private:
        
        Servo servoX;
        Servo servoY;

        double servo_x, servo_y; // These will hold the final servo angles to write
        double servo_x_raw, servo_y_raw; // These will hold the raw PID outputs before constraining

        double setpoint; 
        
        double inputX, outputX;
        double inputY, outputY; // Not used in this version but can be added later for more control
        double inputZ, outputZ; 

        PID pidX;
        PID pidY; // Not used in this version but can be added later for more control
        PID pidZ; 

        Kalman kalmanX;
        Kalman kalmanY; // Not used in this version but can be added later for more control
        Kalman kalmanZ;
};

#endif
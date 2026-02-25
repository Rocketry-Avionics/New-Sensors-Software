#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

// Board Selection
#define HW_TARGET_TEENSY_41
// #define HW_TARGET_PORTENTA_H7

// Pin Definitions
#ifdef HW_TARGET_TEENSY_41
    const int PIN_SERVO_X = 24;
    const int PIN_SERVO_Y = 25;
    const int PIN_CHUTE_SERVO = 26;


#elif defined(HW_TARGET_PORTENTA_H7)
    // Portenta constants
    
#endif


const double DEADBAND = 1.0; // Degrees of tolerance before PID kicks in

// Parachute Deployment Thresholds and Constraints (all in metres)
const float APOGEE_DROP_THRES = 2.0; 
const float MIN_ARM_ALTITUDE = 60.0; 

// PID Gains
const double PID_KP = 2.0;
const double PID_KI = 0.0;
const double PID_KD = 2.5;

// Complementary Filter Weight
const float FILTER_ALPHA = 0.95; 

// Kalman Filter Constants
const double KALMAN_R = 40.0;
const double KALMAN_H = 1.0;
const double KALMAN_Q = 10.0;


const long LORA_FREQUENCY = 0;

#endif
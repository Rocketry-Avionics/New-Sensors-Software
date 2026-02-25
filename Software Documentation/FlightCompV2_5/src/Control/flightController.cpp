#include "FlightController.h"

// 1. Constructor with Member Initializer List
// This is where we wire the PID objects to our variables and config constants!
FlightController::FlightController():
    setpoint(0.0), inputX(0.0), outputX(0.0),
    inputY(0.0), outputY(0.0),
    inputZ(0.0), outputZ(0.0),

    pidX(&inputX, &outputX, &setpoint, PID_KP, PID_KI, PID_KD, DIRECT),
    pidY(&inputY, &outputY, &setpoint, PID_KP, PID_KI, PID_KD, DIRECT),
    pidZ(&inputZ, &outputZ, &setpoint, PID_KP, PID_KI, PID_KD, DIRECT),

    kalmanX(),
    kalmanY(), // Not used in this version but can be added later for more control
    kalmanZ()
    
{
    
}


void FlightController::init() {

    servoX.attach(PIN_SERVO_X); //Check +Z and -X wrt mpu // X servo wire connects to lower pins of pcb when flight commputer placed in rocket with ground on right side(brown)
    servoY.attach(PIN_SERVO_Y); //Check +Z and +X wrt mpu // Y servo wire connects to upper pins of pcb when flight commputer placed in rocket
    
    servoX.write(90); 
    servoY.write(153);


    pidX.SetMode(AUTOMATIC); 
    pidY.SetMode(AUTOMATIC);
    pidZ.SetMode(AUTOMATIC); // Not used in this version but can be added later for more control

    pidX.SetOutputLimits(-90, 90);
    pidY.SetOutputLimits(-90, 90);
    pidZ.SetOutputLimits(-90, 90); // Not used in this version but can be added later for more control


    return;
}


void FlightController::update(double raw_diff_x, double raw_diff_z, double raw_diff_y) {

    inputX = kalmanX.filter(raw_diff_x);
    inputY = kalmanY.filter(raw_diff_y);
    inputZ = kalmanZ.filter(raw_diff_z);

    if (fabs(inputX) < DEADBAND && fabs(inputZ) < DEADBAND) {
        servo_x = 90;
        servo_y = 90;
    }
    else {
        pidX.Compute();
        // pidY.Compute(); // Not used in this version but can be added later for more control
        pidZ.Compute();

        double raw_servo_x = 90 - (0.707 * (outputZ - outputX));
        double raw_servo_y = 90 - (0.707 * (outputZ + outputX));

        // 2. CONSTRAIN THEM (This fixes the 217 / -68 issue)
        servo_x = constrain(raw_servo_x, 0, 180);
        servo_y = constrain(raw_servo_y, 0, 180);
    }


    servoX.write(servo_x);
    servoY.write(servo_y);
}

void FlightController::printDebug() const {
    Serial.print("\nPID Output X: ");
    Serial.print(outputX);
    Serial.print("\nPID Output Z: ");
    Serial.print(outputZ);
    Serial.print("\nKalman Filtered X: ");
    Serial.print(inputX);
    Serial.print("\nKalman Filtered Z: ");
    Serial.print(inputZ);


    Serial.print("\nServo X angle: ");
    Serial.print(servo_x);
    Serial.print("\nServo Y angle: ");
    Serial.println(servo_y);

}
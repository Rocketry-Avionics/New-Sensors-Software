#include "kalman.h"

Kalman::Kalman() {
    P = 1.0; // Initial estimate error covariance
    U_hat = 0.0; // Initial state estimate
    K = 0.0; // Kalman Gain
}


double Kalman::filter(double rawVal) {

    K = P * KALMAN_H / (KALMAN_H * P * KALMAN_H + KALMAN_R); // Calculate Kalman Gain
    U_hat += K * (rawVal - KALMAN_H * U_hat); // Update estimate with measurement
    P = (1 - K * KALMAN_H) * P + KALMAN_Q; // Update estimate error covariance

    return U_hat;
}
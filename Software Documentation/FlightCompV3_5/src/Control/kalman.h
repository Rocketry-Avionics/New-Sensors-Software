#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "../Config/flightConfig.h"


class Kalman {
    public:
        Kalman();

        double filter(double rawVal);

    private:
        double P;
        double U_hat;
        double K;
        
};  



#endif
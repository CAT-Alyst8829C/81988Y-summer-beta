#ifndef PID_H
#define PID_H
#include "drivetrain/odom.hpp"

double lateralIntegral_ = 0;
double lateralPrevError_ = 0;
double angularIntegral_ = 0;
double angularPrevError_ = 0;



double calculateLateral(double error, double kP, double kI, double kD) {
    // Calculate Integral
    lateralIntegral_ += error;
    if ((error / fabs(error)) != (lateralPrevError_ / fabs(lateralPrevError_))) {
        lateralIntegral_ = 0;
    }

    if (lateralIntegral_ > 2000) {
        lateralIntegral_ = 0;
    }
    

    // Calculate Derivative
    const double derivative = error - lateralPrevError_;
    lateralPrevError_ = error;
    
    // Calculate Output
    return error * kP + lateralIntegral_ * kI + derivative * kD;

}

double calculateAngular(double error, double kP, double kI, double kD) {
    // Calculate Integral
    angularIntegral_ += error;
    if ((error / fabs(error)) != (angularPrevError_ / fabs(angularPrevError_))) {
        angularIntegral_ = 0;
    }

    // Calculate Derivative
    const double derivative = error - angularPrevError_;
    angularPrevError_ = error;
    
    // Calculate Output
    return error * kP + angularIntegral_ * kI + derivative * kD;
}


void resetLateralPID() {
    lateralIntegral_ = 0;
    lateralPrevError_ = 0;
}

void resetAngularPID() {
    angularIntegral_ = 0;
    angularPrevError_ = 0;
}



#endif // !PID_H

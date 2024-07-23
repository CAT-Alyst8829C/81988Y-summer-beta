#ifndef OP_H
#define OP_H

#include "drivetrain/dtconfig.hpp"


void setDrive(double l, double r) {
    lB.spin(fwd, l, voltageUnits::mV);
    lM.spin(fwd, l, voltageUnits::mV);
    lF.spin(fwd, l, voltageUnits::mV);

    rB.spin(fwd, r, voltageUnits::mV);
    rM.spin(fwd, r, voltageUnits::mV);
    rF.spin(fwd, r, voltageUnits::mV);
}

void arcade(double throttle, double turn) {
    double throttleV = throttle / 100 * 12000;
    double turnV = turn / 100 * 12000;

    setDrive(throttleV + turnV, throttleV - turnV);
}

#endif 
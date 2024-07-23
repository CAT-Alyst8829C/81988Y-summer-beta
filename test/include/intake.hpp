#ifndef INTAKE_H
#define INTAKE_H
#include "pursuit/curve.hpp"
#include "distance.hpp"
motor intake_1(PORT20, gearSetting::ratio6_1, 1);
motor intake_2(PORT11, gearSetting::ratio6_1, 1);


void setIntakeMotors() {

    intake_1.spin(fwd, 12000 * (master.ButtonR1.pressing() - master.ButtonR2.pressing()), voltageUnits::mV);
    if (master.ButtonY.PRESSED) {
        if (activate) {
            activate = false;
            master.rumble("-");
        }
        else {
            activate = true;
            master.rumble(".");
        }
    }
    if (master.ButtonA.PRESSED) {
        if (colorActivate) {
            colorActivate = false;
            master.rumble("-");
        }
        else {
            colorActivate = true;
            master.rumble(".");
        }
    }
    if (!lock && !colorDiff) {
        intake_2.spin(fwd, 12000 * (master.ButtonL1.pressing() - master.ButtonL2.pressing()), voltageUnits::mV);
    }

    if (lock) {intake_2.spin(fwd, 0, voltageUnits::mV);}

    if (master.ButtonL1.PRESSED) {
        lock = false;
    }
    

}

#endif // !INTAKE_H
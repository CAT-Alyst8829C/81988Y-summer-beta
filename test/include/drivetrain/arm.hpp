#ifndef ARM_H
#define ARM_H

#include "pursuit/follow.hpp"

motor armLift(PORT19, gearSetting::ratio18_1, true);

void mogoArm() {
    // if (master.ButtonUp.PRESSED) {
    //     armLift.spinToPosition(0, rotationUnits::rev, true);
    // }

    // if (master.ButtonDown.PRESSED) {
    //     armLift.spinToPosition(0.75, rotationUnits::rev, true);
    // }

    armLift.spin(fwd, 12000 * (master.ButtonDown.pressing() - master.ButtonUp.pressing()), voltageUnits::mV);
    
    
}

#endif // !ARM_H
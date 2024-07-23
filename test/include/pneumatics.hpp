#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include "intake.hpp"
digital_out armL(Brain.ThreeWirePort.B);
digital_out armR(Brain.ThreeWirePort.D);
digital_out mogo(Brain.ThreeWirePort.A);
bool armStat = false;
bool mogoStat = false;

void setPneumatics() {
    if (master.ButtonB.PRESSED) {
        if (!armStat) {
            armL.set(true);
            armR.set(true);
            armStat = true;
        }
        else {
            armL.set(false);
            armR.set(false);
            armStat = false;
        }
    }

    if (master.ButtonX.PRESSED) {
        if (!mogoStat) {
            mogo.set(true);
            mogoStat = true;
        }
        else {
            mogo.set(false);
            mogoStat = false;
        }
    }

    
}


#endif // !PNEUMATICS_H

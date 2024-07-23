#ifndef DISTANCE_H
#define DISTANCE_H

#include "pursuit/curve.hpp"
#include "intake.hpp"
#include "vex.h"


vex::distance dis_1(PORT17);
optical optic(PORT14);
bool lock = false;
bool colorDiff = false;
bool activate = true;
bool colorActivate = true;

int detectBottomRing() {
    while (1) {
        if (activate) {
        if (master.ButtonL1.PRESSED) {
            lock = false;
            vexDelay(500);
        }

        if (dis_1.objectDistance(vex::distanceUnits::mm) <= 40) {
            lock = true;
        }

        else {
            lock = false;
        }

        
        
        

        
    }
    vexDelay(10);
    }
    
    return 0;
}





#endif // !DISTANCE_H
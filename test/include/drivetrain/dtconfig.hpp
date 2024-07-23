#ifndef CONFIG_H
#define CONFIG_H

#include "vex.h"


// drivetrain motors
motor lB(PORT2, ratio6_1, 1);
motor lM(PORT3, ratio6_1, 0);
motor lF(PORT4, ratio6_1, 1);

motor rB(PORT8, ratio6_1, 0);
motor rM(PORT9, ratio6_1, 1);
motor rF(PORT10, ratio6_1, 0);

// imu
inertial imu(PORT7);

// tracking wheels
rotation horizontal(PORT1, 1);
rotation vertical(-1, 0);

// Drivetrain Parameters
double backWheelDis = -4.5;
double trackingWidth = 15;
double wheelDiameter = 3.25;
double dt_rpm = 450;

// controller
controller master(primary);

// brain
brain Brain;

#endif
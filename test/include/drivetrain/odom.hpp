#ifndef ODOM_H
#define ODOM_H


#include "drivetrain/pid.hpp"
#include "drivetrain/dtconfig.hpp"


double prevVertical = 0;
double prevVertical1 = 0;
double prevVertical2 = 0;
double prevHorizontal = 0;
double prevHorizontal1 = 0;
double prevHorizontal2 = 0;
double prevImu = 0;
double x = 0;
double y = 0;
double theta = 0;
double angle = 0;
double realTheta = 0;
double actualDeltaHeading = 0;

int track() {
    while (1) {
        // Calculate Heading in Radian
        double imuRaw = 0;

        imuRaw = -imu.rotation(rotationUnits::deg) * (M_PI / 180);
        double deltaImu = imuRaw - prevImu;

        double heading = theta;
        heading += deltaImu;

        double deltaHeading = heading - theta;
        double avgHeading = -(theta - M_PI / 2 + heading - M_PI / 2) / 2;

        actualDeltaHeading = -deltaHeading;

        prevImu = imuRaw;

        

        // Calculate Raw Readings on Encoders and Tracking Wheels
        double leftSum = 0;

        leftSum += lB.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        leftSum += lM.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        leftSum += lF.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        double verticalOffset = -(trackingWidth / 2);
        double rawVertical = leftSum / 3;
        double rawHorizontal = (horizontal.position(rotationUnits::deg) * 3.25 * M_PI / 360);
        double horizontalOffset = backWheelDis;

        // Calculate the Change in X and Y
        double deltaX = 0;
        double deltaY = 0;

        deltaY = rawVertical - prevVertical;
        deltaX = rawHorizontal - prevHorizontal;

        prevVertical = rawVertical;
        prevHorizontal = rawHorizontal;

        // Calculate Local X and Y
        double localX = 0;
        double localY = 0;

        if (deltaHeading == 0) { // prevent divide by 0
            localX = deltaX;
            localY = deltaY;
        }
        else {
            localX = 2 * sin(actualDeltaHeading / 2) * (deltaX / actualDeltaHeading + horizontalOffset);
            localY = 2 * sin(actualDeltaHeading / 2) * (deltaY / actualDeltaHeading + verticalOffset);
        }

        // Calculate Global Coordinate
        x += localY * sin(avgHeading);
        y += localY * cos(avgHeading);

        x += localX * -cos(avgHeading);
        y += localX * sin(avgHeading);
        theta = heading;

        angle = -(theta / (M_PI / 180) - 90);
        vexDelay(10);
    }
    return 0;
}


void calibrate() {
    



    resetLateralPID();
    resetAngularPID();

    double prevVertical = 0;
    double prevVertical1 = 0;
    double prevVertical2 = 0;
    double prevHorizontal = 0;
    double prevHorizontal1 = 0;
    double prevHorizontal2 = 0;
    double prevImu = 0;
    double x = 0;
    double y = 0;
    double theta = 0;
    double angle = 0;
    master.rumble(". .");

}


int screen() {
    while (1)
    {
        Brain.Screen.printAt(20, 50, "x: %f", x);
        Brain.Screen.printAt(20, 100, "y: %f", y);
        Brain.Screen.printAt(20, 150, "theta: %f", angle);
        
        
        vexDelay(100);
        
    }
    return 0;
}









#endif // !ODOM_H









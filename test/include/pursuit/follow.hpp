#ifndef FOLLOW_H
#define FOLLOW_H
#include "distance.hpp"
#include "drivetrain/opcontrol.hpp"
#include <iostream>

void followCurve(Vector2d startPose, Vector2d targetPose, Vector2d controlPoint1, Vector2d controlPoint2, double timeOut, double lateralSpeed, double lookAhead) {
    Curve route = Curve(startPose, controlPoint1, controlPoint2, targetPose);
    double timeCount = 0;
    Vector2d currentPose = Vector2d(x, y);

    Vector2d endLineA = targetPose - controlPoint2;
    Vector2d endLineB;

    while ( (endLineA.dot(endLineB) > 0 || route.findCarrotPoint(route, lookAhead) != 1)) {
        timeCount++;
        currentPose = Vector2d(x, y);
        Vector2d origin = Vector2d(0, 0);
        master.Screen.clearLine();
        Vector2d carrotPoint = route.curveCalc(route.findCarrotPoint(route, lookAhead));

        if (carrotPoint == targetPose) {
            Vector2d direction = targetPose - controlPoint2;
            direction.normalize();
            double d = fabs(lookAhead - (currentPose - targetPose).norm());
            carrotPoint = d * direction + targetPose;
        }

        endLineB = targetPose - currentPose;
        
        double targetTheta = acos((carrotPoint[0] - currentPose[0] )/ (carrotPoint - currentPose).norm()) - theta;
        
        double a = theta - targetTheta;

        if (a > 0) {
            if (fabs(a) > M_PI / 2) {
                a = M_PI / 2;
            } 
        }
        else {
            if (fabs(a) > M_PI / 2) {
                a = -M_PI / 2;
            } 
        }
        double curvature = sin(a);

        double feedTurnForward = 3000;

        double turnPower = curvature * feedTurnForward;

        
        setDrive(lateralSpeed + turnPower, lateralSpeed - turnPower);
        vexDelay(10);
        std::cout<<endLineA.transpose()<<"  " << endLineB.transpose() << std::endl;
        std::cout<< "targetX:"<< carrotPoint[0] << " targetY:"<< carrotPoint[1] << " currentX:" << x << " currentY:" << y << " a:" << a / (M_PI / 180) << " targetTheta:" << targetTheta / (M_PI / 180) << " robotTheta:" << theta / (M_PI / 180) << " curvature:" << curvature << " turnPower:" << turnPower << " t:" << route.findCarrotPoint(route, lookAhead) << " end condition:" << endLineA.dot(endLineB) << std::endl;
        std::cout << " " << std::endl;
    }

    setDrive(0, 0);
    
}





#endif // !FOLLOW_H

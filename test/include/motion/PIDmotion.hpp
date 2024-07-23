#ifndef PIDMOTION_H
#define PIDMOTION_H
#include "drivetrain/pid.hpp"

void turnToPoint(double paramx, double paramy, double timeout, double forwards, double kP, double kI, double kD) {
    resetLateralPID();
    resetAngularPID();

    double timeCount = 0;
    double currentTheta = angle;

    currentTheta = (forwards) ? fmod(currentTheta, 360) : fmod(currentTheta - 180, 360);

    double deltaX = paramx - x;
    double deltaY = paramy - y;

    double targetTheta = fmod((M_PI_2 - atan2(deltaY, deltaX)) / (M_PI / 180), 360);

    double error = targetTheta - currentTheta;

    double direction = 1;

    if (error > 180) {
        direction = -1;
    }
    

    while (fabs(error >= 1) && timeCount <= timeout / 10) {
        currentTheta = (forwards) ? fmod(currentTheta, 360) : fmod(currentTheta - 180, 360);
        error = targetTheta - currentTheta;
        double motorPower = calculateAngular(error, kP, kI, kD);
        setDrive(direction * motorPower, -direction * motorPower);
        timeCount += 1;
        vexDelay(10);
    }
    
    setDrive(0, 0);
}


void turnToHeading(double theta, double timeout, double forwards, double kP, double kI, double kD) {
    resetLateralPID();
    resetAngularPID();

    double timeCount = 0;
    double currentTheta = angle;

    currentTheta = (forwards) ? fmod(imu.rotation(rotationUnits::deg), 360) : fmod(imu.rotation(rotationUnits::deg), 360);


    double targetTheta = theta;

    double error = targetTheta - currentTheta;

    double direction = 1;

    if (error > 180) {
        direction = -1;
    }

    while (timeCount <= timeout / 10) {
        currentTheta = (forwards) ? fmod(imu.rotation(rotationUnits::deg), 360) : fmod(imu.rotation(rotationUnits::deg), 360);
        error = targetTheta - currentTheta;
        double motorPower = calculateAngular(error, kP, kI, kD);
        setDrive(direction * motorPower, -direction * motorPower);
        timeCount += 1;
        vexDelay(10);
    }
    setDrive(0, 0);

    
}

void moveToPoint(double distance, double timeout, double forwards, double kP, double kI, double kD) {
    resetLateralPID();
    resetAngularPID();

    double timeCount = 0;

    double targetDistance = distance;
    double error = targetDistance;

    double angleError = 0;
    double targetAngle = angle;

        // Calculate Raw Readings on Encoders and Tracking Wheels
        double leftSum = 0;

        leftSum += lB.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        leftSum += lM.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        leftSum += lF.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        double deltaVertical = 0;
        double prevVertical = leftSum / 3;
        double rawVertical = leftSum / 3;

        double distanceTraveled = 0;

    while (timeCount <= timeout / 10 && fabs(targetDistance - distanceTraveled) > 0.5) {


        // Calculate Raw Readings on Encoders and Tracking Wheels
        double leftSum = 0;

        leftSum += lB.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        leftSum += lM.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);
        leftSum += lF.position(rotationUnits::rev) * (wheelDiameter * M_PI) * (dt_rpm / 600);

        rawVertical = leftSum / 3;
        deltaVertical = rawVertical - prevVertical;
        distanceTraveled += deltaVertical;
        prevVertical = rawVertical;
        
        error = targetDistance - distanceTraveled;
        angleError = targetAngle - angle;
        double adjustment = calculateAngular(angleError, 185, 1, 80);
        double motorPower = calculateLateral(error, kP, kI, kD);
        setDrive(forwards * (motorPower + adjustment), forwards * (motorPower - adjustment));
        vexDelay(10);
    }

    setDrive(0, 0);
    
    
}

void swingToHeading(double theta, double timeout, double forwards, double kP, double kI, double kD) {
    resetLateralPID();
    resetAngularPID();

    double timeCount = 0;
    double currentTheta = angle;

    currentTheta = (forwards) ? fmod(imu.rotation(rotationUnits::deg), 360) : fmod(imu.rotation(rotationUnits::deg), 360);


    double targetTheta = theta;

    double error = targetTheta - currentTheta;

    double direction = 1;
    double left = 1;
    double right = 1;

    if (error > 180) {
        direction = -1;
    }

    if (direction == -1)
    {
        left = 0;
    }
    else
    {
        right = 0;
    }
    
    

    while (timeCount <= timeout / 10) {
        currentTheta = (forwards) ? fmod(imu.rotation(rotationUnits::deg), 360) : fmod(imu.rotation(rotationUnits::deg), 360);
        error = targetTheta - currentTheta;
        double motorPower = calculateAngular(error, kP, kI, kD);
        setDrive(left * motorPower, right * motorPower);
        timeCount += 1;
        vexDelay(10);
    }
    setDrive(0, 0);

    
}


#endif // !PIDMOTION_H
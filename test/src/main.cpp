/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       qianjunye                                                 */
/*    Created:      7/17/2024, 12:02:26 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "api.hpp"
#include <iostream>
#include "pursuit/follow.hpp"


using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
int detectColor() {

  while (1) {
    if (colorActivate) {
      
        if (optic.color() == vex::color::blue && optic.isNearObject()) {
            colorDiff = true;            
              intake_2.setPosition(0, rotationUnits::deg);
              intake_2.spinToPosition(510, rotationUnits::deg, 200, velocityUnits::rpm, true);
            
            intake_2.spin(fwd, -12000, vex::voltageUnits::mV);
            vexDelay(150);
            intake_2.spin(fwd, 0, vex::voltageUnits::mV);
            colorDiff = false;
            vexDelay(500);
        }
    

    }
    Brain.Screen.printAt(20, 200, "intakePose: %f", intake_2.position(rotationUnits::deg));
    vexDelay(10);
  }
  
    
    return 0;
    
}

void redLeft() {
  x = 0;
  y = 0;
  vexDelay(400);
  // theta = M_PI;
  moveToPoint(-20, 1000, 1, 700, 1, 0);
  mogo.set(true);
  moveToPoint(-8, 500, 1, 1000, 1, 0);
  turnToHeading(90, 1300, 1, 215, 1, 250);
  intake_1.spin(fwd, 12000, voltageUnits::mV);
  intake_2.spin(fwd, 12000, voltageUnits::mV);
  moveToPoint(5, 2000, 1, 700, 1, 0);
  vexDelay(400);
  moveToPoint(15, 1000, 1, 700, 1, 0);
  turnToHeading(180, 1300, 1, 215, 1, 250);
  armLift.spinTo(70, rotationUnits::deg, false);
  swingToHeading(270, 1200, 1, 100, 1, 200);
  vexDelay(200);
  turnToHeading(310, 1000, 1, 215, 0.5, 250);

  

}
void pre_auton(void) {
  calibrate();
  vexDelay(2000);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  // lB.setBrake(vex::brakeType::brake);
  // lM.setBrake(vex::brakeType::brake);
  // lF.setBrake(vex::brakeType::brake);

  // rB.setBrake(vex::brakeType::brake);
  // rM.setBrake(vex::brakeType::brake);
  // rF.setBrake(vex::brakeType::brake);
  intake_2.setBrake(brakeType::hold);
  master.Screen.clearScreen();
  armLift.setBrake(brakeType::hold);
  task odom(track);
  task screenTask(screen);
  task ring1(detectBottomRing);
  task ringColor(detectColor);
  

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  master.Screen.clearScreen();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
  

  x = 0;
  y = 0;
  theta = 90 * (M_PI / 180);
  armLift.setPosition(0, rotationUnits::deg);
  // vexDelay(1000);
  // followCurve(Vector2d(0, 0), Vector2d(40, 60), Vector2d(17.2, 48), Vector2d(0, 60), 30000, 5000, 10);

  // moveToPoint(10, 1200, 500, 1, 0);
  redLeft();
  // armLift.spinTo(470, rotationUnits::deg, 200, velocityUnits::rpm, false);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    double leftY = master.Axis3.position();
    double rightX = master.Axis1.position();
    setIntakeMotors();
    setPneumatics();
    arcade(leftY, rightX);
    mogoArm();
    
    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  optic.setLight(ledState::on);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  
  // Run the pre-autonomous function.
  pre_auton();
  // x = 9.96312;
  // y = 8.88012;
  // Curve testC = Curve(Vector2d(0, 0), Vector2d(5, 5), Vector2d(10, 2), Vector2d(10, 10));

  // double testT = testC.findCarrotPoint(testC, 5);

  // Vector2d testCoord = testC.curveCalc(testT);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Admin                                                     */
/*    Created:      9/10/2024, 11:25:01 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using std::abs;

// A global instance of competition
competition Competition = competition();

void usercontrol(void) {
  Controller1.ButtonR2.pressed([](void){intake.spin(fwd,100,pct);});
  Controller1.ButtonR2.released([](void){intake.stop(coast);});
  Controller1.ButtonA.pressed([](void){pneum.set(!pneum.value());});

  while(1) {
    int y = Controller1.Axis3.position();
    int x = Controller1.Axis1.position();

    if(abs(x) < 10)
      x = 0;
    if(abs(y) < 10)
      y = 0;

    if(y+x != 0)
      leftMotors.spin(fwd,(y+x),pct);
    else
      leftMotors.stop(coast);
    
    if(y-x != 0)
      rightMotors.spin(fwd,(y-x),pct);
    else
      rightMotors.stop(coast);
    wait(5,msec); // So the cortex doesn't overload
  } 
}

void auton_1() {

}

void auton_2() {

}

void (* autonomous)(void);

void pre_auton(void) {

  if(Inertial.installed()) {
    Brain.Screen.printAt(0,10,0,"Inertial is calibrating");
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    Brain.Screen.printAt(0,10,0,"                       ");
  }
  else
    Brain.Screen.printAt(0,10,0,"Inertial is not installed!");

  if(triport(PORT1).installed())
    autonomous = auton_2;
  else
    autonomous = auton_1;
}

int main() {

  pre_auton();
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}

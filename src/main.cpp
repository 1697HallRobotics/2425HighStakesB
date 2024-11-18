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

void drive();
void tank_drive();

void tankdrive(void) {
  Controller1.ButtonR2.pressed([](void){intake.spin(fwd,100,pct);});
  Controller1.ButtonR2.released([](void){intake.stop(coast);});
  Controller1.ButtonR1.pressed([](void){intake.spin(fwd,-100,pct);});
  Controller1.ButtonR1.released([](void){intake.stop(coast);});
  Controller1.ButtonA.pressed([](void){pneum.set(!pneum.value());});

  while(1) {
    int x1 = Controller1.Axis4.position();
    int x2 = Controller1.Axis1.position();


    if(abs(x1) < 10)
      leftMotors.spin(fwd,x1,pct);
    else
      leftMotors.stop(coast);
    
    if(abs(x2) < 10)
      rightMotors.spin(fwd,x2,pct);
    else
      rightMotors.stop(coast);
    wait(5,msec); // So the cortex doesn't overload
  } 
}
void usercontrol(void) {
  //Controller1.ButtonR2.pressed([](void){intake.spin(fwd,100,pct);});
  //Controller1.ButtonR2.released([](void){intake.stop(coast);});
  //Controller1.ButtonR1.pressed([](void){intake.spin(fwd,-100,pct);});
  //Controller1.ButtonR1.released([](void){intake.stop(coast);});
  Controller1.ButtonA.pressed([](void){pneum.set(!pneum.value());});

  while(1) {

    if(Controller1.ButtonR2.pressing())
      intake.spin(fwd,100,pct);
    if(Controller1.ButtonR1.pressing())
      intake.spin(fwd,-100,pct);
    if(!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing())
      intake.stop(coast);

    drive();

    wait(5,msec); // So the cortex doesn't overload
  } 
}

void drive() {
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
}

void tank_drive() {
  
  int y = Controller1.Axis3.position();
  int ry = Controller1.Axis2.position();

  if(abs(ry) < 10)
    ry = 0;
  if(abs(y) < 10)
    y = 0;
  
  if(y != 0)
      leftMotors.spin(fwd,y,pct);
    else
      leftMotors.stop(coast);
    
    if(ry != 0)
      rightMotors.spin(fwd,ry,pct);
    else
      rightMotors.stop(coast);


}

void auton_1() {
  thread T([] {
    timer Timer = timer();
    while(Timer.time(msec) < 100) {
      leftMotors.spin(fwd,70,pct);
      rightMotors.spin(fwd,70,pct);
    }
  });
  intake.spinFor(2000,msec,200,rpm);
  T.detach();
}

/*
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

  if(triport(PORT19).installed())
    autonomous = auton_1;
  else
    autonomous = auton_1;
}

*/

timer Timer;
void move(float leftpow, float rightpow, float tmsec, bool delay = true) {
  leftMotors.spin(fwd,leftpow,pct);
  rightMotors.spin(fwd,rightpow,pct); 
  if(delay)
    Timer.event([] {allMotors.stop(hold);},tmsec);
  else {
    wait(tmsec,msec);
    allMotors.stop(hold);
  }
}

void auton() {
  move(70,70,1000);
  move(70,-70,600);
  move(-60,-60,200);
  pneum.close();
  move(-80,-60,800);
}



int main() {

  //pre_auton();
  
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}

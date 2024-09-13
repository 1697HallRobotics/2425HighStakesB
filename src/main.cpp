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

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1 = controller();
controller Controller2 = controller(); // Maybe dual controller
//drive motors
motor left_front = motor(PORT1, ratio6_1, 0); // 11w
motor left_back = motor(PORT2, ratio6_1, 0); // 11w
motor right_front = motor(PORT3, ratio6_1, 0); // 11w
motor right_back = motor(PORT4, ratio6_1, 0); // 11w
motor_group leftMotors = motor_group(left_front, left_back);
motor_group rightMotors = motor_group(right_front, right_back);
//pneumatics
digital_out pneum = digital_out(Brain.ThreeWirePort.A);
//mogo motor
motor mogoMotor = motor(PORT5, ratio18_1, 0); // 11w ? 5.5w
//intake motors
motor roller = motor(PORT6, ratio18_1, 0); // 5.5w ? 5.5w + 5.5w
motor flipper = motor(PORT7, ratio18_1, 0); // 5.5w
//dr4b lift
motor dr4b = motor(PORT8, ratio36_1, 0); // 11w
motor grabber = motor(PORT9, ratio18_1, 0); //11w ? 5.5 + 5.5
//sensors
inertial Inertial = inertial(PORT10);
vision Vision = vision(PORT11); // Unlikely

void usercontrol(void) {

  Controller1.ButtonR2.pressed([](void){roller.spin(fwd,100,percent);});
  Controller1.ButtonR2.released([](void){roller.stop(coast);});
  Controller1.ButtonL2.pressed([](void){grabber.spinFor(60,degrees); grabber.stop(hold);}); // TBD
  Controller1.ButtonL2.released([](void){grabber.stop(coast);});
  Controller1.Axis2.changed(lift); //or create a thread with lift in while loop

  while(1) {
    int x = Controller1.Axis4.position();
    int y = Controller1.Axis3.position();

    if(abs(x) < 10)
      x = 0;
    if(abs(y) < 10)
      y = 0;

    if(y+x != 0)
      leftMotors.spin(fwd,(y+x),percent);
    else
      leftMotors.stop(coast);
    
    if(y-x != 0)
      rightMotors.spin(fwd,(y-x),percent);
    else
      rightMotors.stop(coast);
    wait(5,msec); // So the cortex doesn't overload
  } 
}

void lift() {
  int y = Controller1.Axis2.position();
  if(y > 10)
    dr4b.spin(fwd,y,percent);
  else
    dr4b.stop(coast);
}

bool mogo_eaten = false;

void eat_mogo() {
  if(!mogo_eaten) {
    pneum.set(1);
    mogoMotor.spinFor(100,degrees); // TBD
  }
  else {
    mogoMotor.spinFor(-100,degrees); //TBD
    pneum.set(0);
  }
}

float target = 0, curr = 0;
float error = 0, integral = 0, derivative = 0;
float preverror = 0;
void pid() {
  while(1) {
    //curr = 
    error = target - curr;
    derivative = (preverror - error);
    integral = (error + integral);
    preverror = error;


    wait(15,msec);
  }
  
}

void auton_1() {

}

void auton_2() {

}

void (* autonomous)(void);

void pre_auton(void) {

  mogoMotor.resetPosition();

  if(Inertial.installed()) {
    Inertial.calibrate();
    while(Inertial.isCalibrating()) {
      Brain.Screen.printAt(0,10,0,"Inertial is calibrating.  ");
      wait(200,msec);
      Brain.Screen.printAt(0,10,0,"Inertial is calibrating.. ");
      wait(200,msec);
      Brain.Screen.printAt(0,10,0,"Inertial is calibrating...");
      wait(200,msec);
    }
    Brain.Screen.printAt(0,10,0,"                          ");
  }
  else
    Brain.Screen.printAt(0,10,0,"Inertial is not installed!");

  if(triport(PORT1).installed())
    autonomous = auton_1;
  else
    autonomous = auton_2;
}

int main() {

  pre_auton();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}

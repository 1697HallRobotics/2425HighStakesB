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
brain Brain = brain();
controller Controller1 = controller();
controller Controller2 = controller(); // Maybe dual controller
//drive motors
motor left_front = motor(PORT1, ratio6_1, 0); // 11w
motor left_back = motor(PORT2, ratio6_1, 1); // 11w
motor right_front = motor(PORT3, ratio6_1, 1); // 11w
motor right_back = motor(PORT4, ratio6_1, 0); // 11w
motor_group leftMotors = motor_group(left_front, left_back);
motor_group rightMotors = motor_group(right_front, right_back);
//pneumatics
digital_out pneum = digital_out(Brain.ThreeWirePort.A);
//mogo motor
motor mogoMotor = motor(PORT5, ratio18_1, 0); // 11w ? 5.5w
//intake motors
motor roller = motor(PORT6, ratio18_1, 0); // 5.5w
motor flipper = motor(PORT7, ratio18_1, 0); // 5.5w
//dr4b lift
motor dr4b = motor(PORT8, ratio36_1, 0); // 11w
motor grabber = motor(PORT9, ratio18_1, 0); //11w ? 5.5 + 5.5
//sensors
inertial Inertial = inertial(PORT10);
rotation Rot = rotation(PORT11);
vision Vision = vision(PORT11); // Unlikely

class PIDCtrl {
  private:
    float kP, kI, kD, error, integral, derriv, prevError, (* sense_val) (void);
    int delay, integral_threshold;
  public:
    void init_PID(float kP, float kI, float kD, float(*callback)(void), int delay = 15) {
      this->kP = kP;
      this->kI = kI;
      this->kD = kD;
      this->delay = delay;
      this->integral_threshold = 1000000;
      this->integral = 0;
      this->sense_val = callback;
    }
    float update(float target){
      this->error = target - sense_val();
      if(fabsf(error) < integral_threshold)
        integral += error;
      else 
        integral = 0;
      derriv = error - prevError;
      prevError = error;
      return kP*error + kI*integral + kD*derriv;
    }

};

void trap_acc_drive() {
  float k = 100;
  while(1) {
    int y = Controller1.Axis3.position();
    int x = Controller1.Axis1.position();

    if(abs(x) < 10)
      x = 0;
    if(abs(y) < 10)
      y = 0;
    float leftVel = leftMotors.velocity(pct);
    float rightVel = rightMotors.velocity(pct);
    leftVel += k*((x+y > leftVel) - (x+y < leftVel));
    rightVel += k*((x-y > rightVel) - (x-y < rightVel));
    if(abs(leftVel) > 0)
      leftMotors.spin(fwd,leftVel,pct);
    else
      leftMotors.stop(coast);
    
    if(abs(rightVel) > 0)
      rightMotors.spin(fwd,rightVel,pct);
    else
      rightMotors.stop(coast);

    wait(25,msec); // So the cortex doesn't overload
  } 
}

void pid_drive() {
  PIDCtrl PIDLateral;
  PIDLateral.init_PID(0.0f,0.0f,0.0f,[](void) -> float{return Rot.velocity(rpm);});
  float lateral = 0;

  PIDCtrl PIDRot;
  PIDLateral.init_PID(0.0f,0.0f,0.0f,[](void) -> float{return Inertial.gyroRate(zaxis,rpm);});
  float turn = 0;
 
  while(1) {
    int y = Controller1.Axis3.position()*600;
    int x = Controller1.Axis1.position()*600;
  
    lateral = PIDLateral.update(y);
    turn = PIDRot.update(x);

    leftMotors.spin(fwd, lateral + turn, rpm);
    rightMotors.spin(fwd, lateral - turn, rpm);
    wait(25,msec);
  }
}

void lift() {
  int y = Controller1.Axis2.position();
  if(y > 10)
    dr4b.spin(fwd,y*0.5,pct);
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

void usercontrol(void) {
  Controller1.ButtonR2.pressed([](void){roller.spin(fwd,100,pct);});
  Controller1.ButtonR2.released([](void){roller.stop(coast);});
  Controller1.ButtonL2.pressed([](void){grabber.spinFor(60,degrees); grabber.stop(hold);}); // TBD
  Controller1.ButtonL2.released([](void){grabber.stop(coast);});
  Controller1.Axis2.changed(lift); //or create a thread with lift in while loop

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

float trap_target(int acc_msec, int time, float t) {
  if(t < 0 || t > time)
    return 0;
  if(t <= acc_msec)
    return t/acc_msec;
  if(t >= time - acc_msec)
    return (time-t)/acc_msec;
  return 1;
}
 

void pid_cm(float dist = 0, float turn = 0, float maxspeed = 1) {
  timer Timer = timer();
  PIDCtrl drivePID;
  float acc_msec = 2;
  drivePID.init_PID(0.0f,0.0f,0.0f,[](void) -> float{return Inertial.gyroRate(yaxis,rpm);});
  //drivePID.init_PID(0.0f,0.0f,0.0f,[](void) -> float{return Rot.velocity(rpm);});
  float time = dist/(600*maxspeed) + acc_msec;
  float power = 0;
  while(1) {
    power = drivePID.update(trap_target(acc_msec,time,Timer.time(sec)));
    leftMotors.spin(fwd,power,rpm);
    rightMotors.spin(fwd,power,rpm);
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
    Brain.Screen.printAt(0,10,0,"Inertial is calibrating");
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    Brain.Screen.printAt(0,10,0,"                       ");
  }
  else
    Brain.Screen.printAt(0,10,0,"Inertial is not installed!");

  if(triport(PORT1).installed())
    autonomous = auton_1;
  else
    autonomous = auton_2;
}

int main() {

  //pre_auton();
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}

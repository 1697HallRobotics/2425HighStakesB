#include "vex.h"
#include "auton.h"

using namespace vex;

/* 
class PIDCtrl {
  private:
    float kP, kI, kD, error, integral, derriv, prevError, (* sense_val) (void);
    int delay, integral_threshold;
  public:
    PIDCtrl(float kP, float kI, float kD, float(*callback)(void), int delay = 15) {
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

float get_avg_rpm() {
    return 0.25*(left_front.velocity(pct) + right_front.velocity(pct) + left_back.velocity(pct) + right_back.velocity(pct));
}

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


float trap_target(int acc_msec, int time, float t) {
  if(t < 0 || t > time)
    return 0;
  if(t <= acc_msec)
    return t/acc_msec;
  if(t >= time - acc_msec)
    return (time-t)/acc_msec;
  return 1;
}

/*
void pid_drive() {
  PIDCtrl lateral_pid(0.0f,0.0f,0.0f,velocity);
  //PIDCtrl lateral_pid(0.0f,0.0f,0.0f,[](void) -> float{return Rot.velocity(rpm);});
  float lateral = 0;

  PIDCtrl rot_pid(0.0f,0.0f,0.0f,[](void) -> float{return Inertial.angle(deg);});
  float turn = 0;
 
  while(1) {
    int y = Controller1.Axis3.position()*600;
    int x = Controller1.Axis1.position()*600;
  
    lateral = lateral_pid.update(y);
    turn = rot_pid.update(x);

    leftMotors.spin(fwd, lateral - turn, rpm);
    rightMotors.spin(fwd, lateral + turn, rpm);
    wait(25,msec);
  }
}
*/
/*
void drive(float cm) {
  timer Timer = timer();
  PIDCtrl drive_pid(0.0f,0.0f,0.0f,velocity);
  //PIDCtrl drive_pid(0.0f,0.0f,0.0f,[](void) -> float{return Rot.velocity(rpm);});
  float acc_msec = 2;
  float time = 3.25*cm/600 + acc_msec;
  float lateral, rot;
  while(1) {
    lateral = drive_pid.update(600*trap_target(acc_msec,time,Timer.time(sec)));
    leftMotors.spin(fwd,lateral,rpm);
    rightMotors.spin(fwd,lateral,rpm);
  }
}
*/

/*
void trap_drive(float rot) {
  timer Timer = timer();
  bool acc = true;
  float start = Timer.system();
  float K = 0.1; //pct per msec ~~ 100 pct per 1000 msec;
  float dt, K, lateral;
  Timer.clear();
  while(start + 2000 > Timer.system()) {
    dt = Timer.time(msec); // msec
    lateral += dt*K*(acc*2-1);
    if(lateral >= 100)
      lateral = 100;
    if(lateral <= -100)
      lateral = -100;
    if(abs(lateral) > 10) {
      leftMotors.spin(fwd,lateral,rpm);
      rightMotors.spin(fwd,lateral,rpm);
    }
    Timer.clear();
    wait(10,msec);
  }
}

void drive(float cm) {

  timer Timer = timer();
  float acc_msec = 500;
  float time = 3.25*cm/600 + acc_msec;
  float lateral;
  while(1) {
    lateral = 600*trap_target(acc_msec,time,Timer.time(sec));
    leftMotors.spin(fwd,lateral,rpm);
    rightMotors.spin(fwd,lateral,rpm);
  }
}
void turn(float degf) {

  timer Timer = timer();
  float acc_msec = 500;
  float time = 3.25*degf/600 + acc_msec;
  float lateral;
  while(1) {
    lateral = 600*trap_target(acc_msec,time,Timer.time(sec));
    leftMotors.spin(fwd,-lateral,rpm);
    rightMotors.spin(fwd,lateral,rpm);
  }
}
*/
/*
void turn(float degf) {
  //PIDCtrl rot_pid(0.0f,0.0f,0.0f,[](void) -> float{return Inertial.gyroRate(zaxis,rpm);});
  PIDCtrl rot_pid(0.0f,0.0f,0.0f,[](void) -> float{return Inertial.angle(deg);});

  float turn;

  while(1) {
    turn = rot_pid.update(degf);
    leftMotors.spin(fwd,turn,pct);
    rightMotors.spin(fwd,-turn,pct);
  }
}
*/
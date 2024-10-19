#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

extern brain Brain;
extern controller Controller1;
extern controller Controller2;
//drive motors
extern motor left_front;
extern motor left_back;
extern motor right_front;
extern motor right_back;
extern motor_group leftMotors;
extern motor_group rightMotors;
//pneumatics
extern digital_out pneum;
//intake motors
extern motor_group intake;
//sensors
extern inertial Inertial;
extern rotation Rot;
extern vision Vision;
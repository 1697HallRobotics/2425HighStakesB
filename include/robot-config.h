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
extern motor intake;
//sensors
extern inertial Inertial;
extern rotation Rot;
extern vision Vision;

float right_velocity();
float left_velocity();
float velocity();

float right_velocity(velocityUnits vU = rpm) {
    return 0.5*(right_front.velocity(vU) + right_back.velocity(vU));
}
float left_velocity(velocityUnits vU = rpm) {
    return 0.5*(left_front.velocity(vU) + left_back.velocity(vU));
}
float velocity(velocityUnits vU = rpm) {
    return 0.25*(left_front.velocity(vU) + left_back.velocity(vU) + right_front.velocity(vU) + right_back.velocity(vU));
}
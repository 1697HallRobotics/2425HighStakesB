#include "vex.h"
#include "robot-config.h"

using namespace vex;

// A global instance of competition
brain Brain = brain();
controller Controller1 = controller();
controller Controller2 = controller(); // Maybe dual controller
//drive motors
motor left_front = motor(PORT5, ratio6_1, 1); // 11w
motor left_back = motor(PORT9, ratio6_1, 1); // 11w
motor right_front = motor(PORT10, ratio6_1, 0); // 11w
motor right_back = motor(PORT8, ratio6_1, 0); // 11w
motor_group leftMotors = motor_group(left_front, left_back);
motor_group rightMotors = motor_group(right_front, right_back);
//pneumatics
pneumatics pneum = pneumatics(Brain.ThreeWirePort.A);
//intake motors
motor floater = motor(PORT11, ratio6_1, 1);
motor hook = motor(PORT1, ratio18_1, 0); // 5.5w
motor_group intake = motor_group(floater, hook);
//sensors
inertial Inertial = inertial(PORT10);
rotation Rot = rotation(PORT11);
vision Vision = vision(PORT11); // Unlikely

#include "vex.h"
#include "robot-config.h"

using namespace vex;

// A global instance of competition
brain Brain = brain();
controller Controller1 = controller();
controller Controller2 = controller(); // Maybe dual controller
//drive motors
motor left_front = motor(PORT7, ratio6_1, 1); // 11w
motor left_back = motor(PORT5, ratio6_1, 1); // 11w
motor right_front = motor(PORT8, ratio6_1, 0); // 11w
motor right_back = motor(PORT19, ratio6_1, 0); // 11w
motor_group leftMotors = motor_group(left_front, left_back);
motor_group rightMotors = motor_group(right_front, right_back);
motor_group allMotors = motor_group(right_front, right_back,left_front, left_back);

//pneumatics
pneumatics pneum = pneumatics(Brain.ThreeWirePort.C);
//intake motors
motor floater = motor(PORT1, ratio6_1, 0);
motor hook = motor(PORT15, ratio18_1, 0); // 5.5w
motor_group intake = motor_group(floater, hook);
//sensors
inertial Inertial = inertial(PORT21);
rotation Rot = rotation(PORT21);
vision Vision = vision(PORT21); // Unlikely

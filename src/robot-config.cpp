#include "vex.h"
#include "robot-config.h"

using namespace vex;

// A global instance of competition
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
//intake motors
motor intake = motor(PORT6, ratio18_1, 0); // 5.5w
//sensors
inertial Inertial = inertial(PORT10);
rotation Rot = rotation(PORT11);
vision Vision = vision(PORT11); // Unlikely

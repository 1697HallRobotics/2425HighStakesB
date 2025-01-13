#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#define LEFT_MOTOR_PORTS {7, 5}
#define RIGHT_MOTOR_PORTS {8, 19}
#define PNEUMATIC_PORT 'a'
#define INTAKE_PORTS {1, 15}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous or opcontrol
 */
void competition_initialize() {}

/**
 */
void autonomous() {}

/**
 */
void opcontrol() {
	int32_t DEADZONE = 10;
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg(LEFT_MOTOR_PORTS);
	pros::MotorGroup right_mg(RIGHT_MOTOR_PORTS);
	pros::MotorGroup intake_mg(INTAKE_PORTS);
	pros::adi::Pneumatics pneumatic(PNEUMATIC_PORT,0);
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


	while (1) {
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		
		if(abs(dir) < DEADZONE)
			dir = 0;
		if(abs(turn) < DEADZONE)
			turn = 0;
		if(dir && turn == 0) {
			left_mg.brake();
			right_mg.brake();
		} else {
			left_mg.move(dir - turn);                      // Sets left motor voltage
			right_mg.move(dir + turn);  					// Sets right motor voltage
		}

		if(master.get_digital(DIGITAL_R2))
			intake_mg.move(127);
		if(master.get_digital(DIGITAL_R1))
			intake_mg.move(-127);
		if(master.get_digital(DIGITAL_R1) == master.get_digital(DIGITAL_R2))
			intake_mg.brake();
		if(master.get_digital(DIGITAL_L1))
			pneumatic.extend();
		if(master.get_digital(DIGITAL_L2))
			pneumatic.retract();
		if(master.get_digital(DIGITAL_A))
			pneumatic.set_value(pneumatic.toggle());

		pros::delay(5);
	}
}
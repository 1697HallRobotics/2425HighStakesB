#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/screen.hpp"

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
void initialize() {
	/*
	pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(6,6,474,133);
	pros::screen::set_pen(pros::Color::blue);
	pros::screen::fill_rect(6,139,474,266);
	pros::screen::set_pen(pros::Color::black);
	pros::screen::fill_rect(237, 133, 243, 139);
	*/
}

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
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(5);                               // Run for 20 ms then update

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

	}
}
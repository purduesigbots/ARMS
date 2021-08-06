#include "main.h"

pros::Controller master(CONTROLLER_MASTER);

arms::chassis::OdomChassis chassis;

void initialize() {
	arms::pid::PID pid = arms::pid::PIDBuilder()
	                         .withLinearPID(0.5, 4, 1.2)
	                         .withAngularPID(0.5, 4, 1.2)
	                         .withLinearPointPID(0.5, 4, 1.2)
	                         .withAngularPID(0.5, 4, 1.2)
	                         .withArcKP(0.5)
	                         .withDifKP(0.5)
	                         .withMinError(1)
	                         .build();

	arms::chassis::Odom odom =
	    arms::chassis::Odom().withDistances(6.375, 5.75).withTPI(41.4, 41.4);

	arms::selector::init();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	chassis.move({24, 0});
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		chassis.arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		               master.get_analog(ANALOG_RIGHT_X) * (double)100 / 127);

		pros::delay(10);
	}
}

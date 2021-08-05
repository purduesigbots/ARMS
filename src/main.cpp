#include "main.h"
#include "ARMS/pid.h"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	arms::pid::PIDBuilder()
	    .withLinearPID(0.5, 4, 1.2)
	    .withAngularPID(0.5, 4, 1.2)
	    .withLinearPointPID(0.5, 4, 1.2)
	    .withAngularPID(0.5, 4, 1.2)
	    .withArcKP(0.5)
	    .withDifKP(0.5)
	    .withMinError(1);

	arms::chassis::init();
	arms::odom::init();
	arms::selector::init();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	arms::odom::move({24, 0});
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		arms::chassis::arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                      master.get_analog(ANALOG_RIGHT_X) * (double)100 /
		                          127);

		pros::delay(10);
	}
}

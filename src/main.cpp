#include "main.h"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	chassis::init();
	odom::init();
	selector::init();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	std::vector<std::array<double, 2>> path1{
	    {72, 0}, {96, -24}, {72, -48}, {36, -24}};

	// purepursuit::followPath(path1);

	odom::goToPoint({72, 0});
	odom::goToPoint({96, -24});
	odom::goToPoint({72, -48});
	odom::goToPoint({36, -24});
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		chassis::arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                master.get_analog(ANALOG_RIGHT_X) * (double)100 / 127);

		pros::delay(20);
	}
}

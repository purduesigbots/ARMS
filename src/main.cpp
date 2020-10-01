#include "main.h"
#include "pros/imu.hpp"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	initDrive();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		       master.get_analog(ANALOG_RIGHT_X) * (double)100 / 127);

		pros::delay(20);
	}
}

#include "main.h"
#include "greenhat/odom.h"
#include "pros/imu.hpp"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	/*greenhat::*/initDrive();
	selector::init();
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
        
        printf("x: %lf y: %lf angle: %lf\n", odom::global_x, odom::global_y, odom::heading);

		pros::delay(20);
	}
}

#include "main.h"
#include "pros/imu.hpp"

pros::Controller master(CONTROLLER_MASTER);
pros::Imu imu(12);

void initialize() {
    imu.reset();
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
        
        printf("%f\n", imu.get_rotation());

		arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		       master.get_analog(ANALOG_RIGHT_X) * (double)100 / 127);

		pros::delay(20);
	}
}

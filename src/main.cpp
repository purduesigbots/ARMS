#include "main.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	chassis::init();
	selector::init();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
    chassis::moveHolo(1, 45);
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected()) {
            autonomous();
        }

		chassis::holonomic(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                master.get_analog(ANALOG_RIGHT_X) * (double)100 / 127,
                        master.get_analog(ANALOG_LEFT_X) * (double)100 / 127);

		pros::delay(20);
	}
}

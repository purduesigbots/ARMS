#include "main.h"
#include "ARMS/config.h"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	arms::init();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	using namespace arms::chassis;
	move({{24, 0}}, 50, arms::THRU | arms::ASYNC);
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		arms::chassis::arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                      master.get_analog(ANALOG_RIGHT_X) * (double)100 /
		                          127);

		if(master.get_digital_new_press(DIGITAL_A)) {
			printf("-----------------------------------------\n");
			arms::chassis::turn(0, 50);
		}
		if(master.get_digital_new_press(DIGITAL_B)) {
			printf("-----------------------------------------\n");
			arms::chassis::turn(90, 50);
		}
		if(master.get_digital_new_press(DIGITAL_X)) {
			printf("-----------------------------------------\n");
			arms::chassis::turn(90, 50, arms::TRUE_RELATIVE);
		}
		if(master.get_digital_new_press(DIGITAL_Y)) {
			printf("-----------------------------------------\n");
			arms::chassis::turn(90, 50, arms::RELATIVE);
		}

		if(master.get_digital_new_press(DIGITAL_LEFT)) {
			printf("-----------------------------------------\n");
			arms::chassis::move({24, 24, 45}, 50);
		}
		if(master.get_digital_new_press(DIGITAL_RIGHT)) {
			printf("-----------------------------------------\n");
			arms::chassis::move({24, 24, 45}, 50, arms::RELATIVE);
		}
		if(master.get_digital_new_press(DIGITAL_UP)) {
			printf("-----------------------------------------\n");
			arms::chassis::move({24, 24, 45}, 50, arms::TRUE_RELATIVE);
		}

		if(master.get_digital(DIGITAL_R1)) {
			printf("-----------------------------------------\n");
			arms::chassis::move(24, 50);
		}
		if(master.get_digital_new_press(DIGITAL_L1)) {
			printf("-----------------------------------------\n");
			arms::chassis::move(24, 50, arms::TRUE_RELATIVE);
		}
		pros::delay(10);
	}
}

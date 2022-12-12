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
	arms::Bezier curve = arms::Bezier({15,15},{50,13},{130,54},{140,37});
	arms::follow_bezier(curve, 100);
	
	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		arms::chassis::arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                      master.get_analog(ANALOG_RIGHT_X) * (double)100 /
		                          127);

		pros::delay(10);
	}
}

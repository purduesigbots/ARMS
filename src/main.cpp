#include "main.h"

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
	using namespace arms::purepursuit;

	std::cout << "Starting autonomous" << std::endl;

	followPath({
		{0, 0},
		{24, 0},
		{24, 24}
	}, 24);

	std::cout << "Ending autonomous" << std::endl;
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

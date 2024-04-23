#include "main.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"

pros::Controller master(CONTROLLER_MASTER);

void initialize() {
	arms::init();
	pros::lcd::initialize();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	using namespace arms::chassis;
	arms::odom::setTrackingOffset(0);
	// move({{24, 12}}, 75);
}

void opcontrol() {
	arms::odom::setTrackingOffset(12);

	while (true) {
		using namespace arms;
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		arms::chassis::arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                      master.get_analog(ANALOG_RIGHT_X) * (double)100 /
		                          127);

		pros::lcd::clear_line(1);
		pros::lcd::clear_line(2);
		pros::lcd::clear_line(3);
		pros::lcd::print(1, "X: %lf", odom::getPosition().x);
		pros::lcd::print(2, "Y: %lf", odom::getPosition().y);
		pros::lcd::print(3, "H: %lf", odom::getHeading());
		
		master.print(0, 0, "%.2f, %.2f, %.2f", odom::getPosition().x, odom::getPosition().y, odom::getHeading());

		pros::delay(10);
	}
}
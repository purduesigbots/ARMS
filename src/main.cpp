#include "main.h"

pros::Controller controller(CONTROLLER_MASTER);

void initialize() {

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		arcade(
			controller.get_analog(ANALOG_LEFT_Y)*(double)100/127,
			controller.get_analog(ANALOG_LEFT_X)*(double)100/127
		);

		pros::delay(20);
	}
}

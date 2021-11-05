#include "main.h"

pros::Controller master(CONTROLLER_MASTER);

double tpu = 57.86;

void initialize() {
	arms::chassis::init({14, -13, -18}, {-19, 20, 16}, // motors
	                    600,                           // gearset
	                    tpu, 4.75,                     // TPU
	                    5,         // 12          // setle time
	                    .5, 1,     // linear/angular thresholds
	                    2, 2,      // regular/arc slew
	                    12,        // imu port
	                    {0, 0, 0}, // encoder ports
	                    0,         // expander port
	                    5          // joystick threshold
	);

	delay(5000);

	arms::odom::init(false, 0, 0, tpu, tpu,
	                 false, // holonomic
	                 3      // exit error
	);

	arms::pid::init(false,     // debug output
	                .1, 0, .1, // linear constants 		 .08, .1
	                1, 0, 6,   // angular contants
	                5, 0, 20,  // linear point constants
	                50, 0, 0,  // angular point constants
	                .05,       // arc kp
	                0,         // dif kp
	                5          // min error
	);
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	arms::odom::move({48, -24}, 60, false);
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

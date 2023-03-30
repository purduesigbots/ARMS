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
	using namespace arms;
	odom::reset({0, 0}, 90);


	// reverse and get roller
	chassis::move(-10, 100, REVERSE | TRUE_RELATIVE);


	chassis::move({-9, 8}, 90);
	chassis::move(-6, 90, REVERSE | TRUE_RELATIVE);


	// align and shoot disk 1 + preloads
	chassis::turn(89, 100);




	chassis::move({22, 18}, 90);

	chassis::move(18, 60);
	chassis::move(-6, 90, REVERSE | TRUE_RELATIVE);

	// align and shoot 3 disks
	chassis::turn(120, 100);



	chassis::turn(100, 100);
	chassis::move(10, 90);
	chassis::move(-10, 90, REVERSE | TRUE_RELATIVE);

	chassis::turn(160, 100);
	chassis::move(10, 90);
	chassis::move(-10, 90, REVERSE);

	// align and shoot two disks
	chassis::turn(111, 100);

	chassis::turn(0, 100);
	chassis::move(20, 90);
	chassis::move(-8, 90, REVERSE | TRUE_RELATIVE);
	chassis::turn(-25, 100);
	chassis::move(16, 90);
	chassis::move(-19, 90, REVERSE | TRUE_RELATIVE);
	chassis::turn(118, 100);

}

void debugInfoPrint() {
	arms::Point current = arms::odom::getPosition();
	arms::Point desired = arms::odom::getDesiredPosition();

	double current_heading = arms::odom::getHeading();
	double desired_heading = arms::odom::getDesiredHeading();

	printf("Current: (%f, %f) | Desired: (%f, %f)\n", current.x, current.y,
	       desired.x, desired.y);
}

void opcontrol() {
	arms::chassis::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		arms::chassis::arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		                      master.get_analog(ANALOG_RIGHT_X) * (double)100 /
		                          127);

		if(master.get_digital_new_press(DIGITAL_A)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::turn(0, 50);
		}
		if(master.get_digital_new_press(DIGITAL_B)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::turn(90, 50);
		}
		if(master.get_digital_new_press(DIGITAL_X)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::turn(90, 50, arms::TRUE_RELATIVE);
		}
		if(master.get_digital_new_press(DIGITAL_Y)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::turn(90, 50, arms::RELATIVE);
		}

		if(master.get_digital_new_press(DIGITAL_LEFT)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::move({24, 24, 45}, 50);
		}
		if(master.get_digital_new_press(DIGITAL_RIGHT)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::move({24, 24, 45}, 50, arms::RELATIVE);
		}
		if(master.get_digital_new_press(DIGITAL_UP)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::move({24, 24, 45}, 50, arms::TRUE_RELATIVE);
		}

		if(master.get_digital(DIGITAL_DOWN)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::move({0,0,0}, 50);
		}

		if(master.get_digital(DIGITAL_R1)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::move(24, 50);
		}
		if(master.get_digital_new_press(DIGITAL_L1)) {
			printf("-----------------------------------------\n");
			debugInfoPrint();
			arms::chassis::move(24, 50, arms::TRUE_RELATIVE);
		}
		pros::delay(10);
	}
}

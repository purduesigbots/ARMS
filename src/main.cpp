#include "main.h"

pros::Controller master(CONTROLLER_MASTER);
arms::chassis::Chassis chassis;

void initialize() {
	arms::pid::PID pid = arms::pid::PIDBuilder()
	                         .withLinearPID(0.1, 0, 2)
	                         .withAngularPID(0, 0, 0)
	                         .withLinearPointPID(0, 0, 0)
	                         .withAngularPID(0, 0, 0)
	                         .withArcKP(0)
	                         .withDifKP(0)
	                         .withMinError(5)
	                         .build();

	chassis = arms::chassis::ChassisBuilder()
	              .withMotors({-1, -2}, {4, 5})
	              .withGearset(1200)
	              .withDistanceConstant(20)
	              .withIMU(0)
	              .withPID(pid)
	              .build();

	chassis.startTask();

	// arms::selector::init();
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
	chassis.move(72);
}

void opcontrol() {

	while (true) {
		if (master.get_digital(DIGITAL_LEFT) && !competition::is_connected())
			autonomous();

		chassis.arcade(master.get_analog(ANALOG_LEFT_Y) * (double)100 / 127,
		               master.get_analog(ANALOG_RIGHT_X) * (double)100 / 127);

		pros::delay(10);
	}
}

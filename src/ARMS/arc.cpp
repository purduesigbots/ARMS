#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "ARMS/pid.h"
#include "api.h"
using namespace pros;

namespace arms::chassis {

double prev = 0; // previous chassis speed

void arc(bool mirror, int arc_length, double rad, int max, int type) {
	reset();
	int time_step = 0;
	pid::mode = DISABLE;
	bool reversed = false;

	// reverse the movement if the length is negative
	if (arc_length < 0) {
		reversed = true;
		arc_length = -arc_length;
	}

	// fix jerk bug between velocity movements
	if (type < 2) {
		motorMove(leftMotors, 0, true);
		motorMove(rightMotors, 0, true);
		delay(10);
	}

	while (time_step < arc_length) {

		// speed
		int error = arc_length - time_step;
		double speed = error * pid::arcKP;

		if (type == 1 || type == 2)
			speed = max;

		// speed limiting
		if (speed > max)
			speed = max;
		if (speed < -max)
			speed = -max;

		// prevent backtracking
		if (speed < 0)
			speed = 0;

		speed = slew(speed, accel_step, &prev); // slew

		if (reversed)
			speed = -speed;

		double scaled_speed = speed * rad;

		if (type == 1)
			scaled_speed *= (double)time_step / arc_length;
		else if (type == 2)
			scaled_speed *= fabs(2 * (.5 - (double)time_step / arc_length));
		else if (type == 3)
			scaled_speed *= (1 - (double)time_step / arc_length);

		// assign chassis motor speeds (using velocity)
		motorMove(leftMotors, mirror ? speed : scaled_speed, 1);
		motorMove(rightMotors, mirror ? scaled_speed : speed, 1);

		// increment time step
		time_step += 10;
		delay(10);
	}

	if (type != 1 && type != 2) {
		motorMove(leftMotors, 0, true);
		motorMove(rightMotors, 0, true);
	}
}

void arcLeft(int arc_length, double rad, int max, int type) {
	arc(false, arc_length, rad, max, type);
}

void arcRight(int arc_length, double rad, int max, int type) {
	arc(true, arc_length, rad, max, type);
}

void scurve(bool mirror, int arc1, int mid, int arc2, int max) {

	// first arc
	arc(mirror, arc1, 1, max, 1);

	// middle movement
	velocity(mid, max);

	// final arc
	arc(!mirror, arc2, 1, max, 2);
}

void sLeft(int arc1, int mid, int arc2, int max) {
	scurve(false, arc1, mid, arc2, max);
}

void sRight(int arc1, int mid, int arc2, int max) {
	scurve(true, arc1, mid, arc2, max);
}

void _sLeft(int arc1, int mid, int arc2, int max) {
	scurve(true, -arc1, mid, -arc2, -max);
}

void _sRight(int arc1, int mid, int arc2, int max) {
	scurve(false, -arc1, -mid, -arc2, max);
}

} // namespace arms::chassis

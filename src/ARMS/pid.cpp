#include "api.h"
#include "arms/api.h"

namespace arms::pid {

int mode = DISABLE;
bool debug = false;

// pid constants
double linearKP;
double linearKI;
double linearKD;
double angularKP;
double angularKI;
double angularKD;
double linear_pointKP;
double linear_pointKI;
double linear_pointKD;
double angular_pointKP;
double angular_pointKI;
double angular_pointKD;
double arcKP;
double difKP;
double dif;
double difMax;
double min_error;

// pid targets
double linearTarget = 0;
double angularTarget = 0;
double vectorAngle = 0;
std::array<double, 2> pointTarget{0, 0};

double pid(double error, double* pe, double* in, double kp, double ki,
           double kd) {
	if (debug)
		printf("%.2f\n", error);

	double derivative = error - *pe;
	if ((*pe > 0 && error < 0) || (*pe < 0 && error > 0))
		*in = 0; // remove integral at zero error
	double speed = error * kp + *in * ki + derivative * kd;

	// scale back integral if over max windup
	if (fabs(speed) < 100) {
		*in += error;
	}

	*pe = error;

	return speed;
}

double pid(double target, double sv, double* pe, double* in, double kp,
           double ki, double kd) {
	double error = target - sv;
	return pid(error, pe, in, kp, ki, kd);
}

std::array<double, 2> linear() {
	static double pe = 0; // previous error
	static double in = 0; // integral

	// get position in the x and y directions
	double sv_x = chassis::position();
	double sv_y = chassis::position(true);

	// calculate total displacement using pythagorean theorem
	double sv;
	if (vectorAngle != 0)
		sv = sqrt(pow(sv_x, 2) + pow(sv_y, 2));
	else
		sv = sv_x; // just use the x value for non-holonomic movements

	double speed = pid(linearTarget, sv, &pe, &in, linearKP, linearKI, linearKD);

	speed = chassis::limitSpeed(speed, chassis::maxSpeed);

	// difference PID
	dif = chassis::difference() * difKP;
	dif = chassis::limitSpeed(dif, difMax);

	return {speed - dif, speed + dif};
}

std::array<double, 2> angular() {
	static double pe = 0; // previous error
	static double in = 0; // integral
	double sv = chassis::angle();
	double speed =
	    pid(angularTarget, sv, &pe, &in, angularKP, angularKI, angularKD);
	return {speed, -speed}; // clockwise positive
}

std::array<double, 2> odom() {
	// previous sensor values
	static double pe_lin = 0;
	static double pe_ang = 0;

	double lin_error = odom::getDistanceError(pointTarget); // linear
	double ang_error = odom::getAngleError(pointTarget);    // angular

	// if holonomic, angular error is relative to field, not to point
	if (pid::mode == ODOM_HOLO || pid::mode == ODOM_HOLO_THRU) {
		pid::vectorAngle = ang_error;
		ang_error = pid::angularTarget - ((int)odom::heading_degrees % 360);

		// make sure all turns take most efficient route
		if (ang_error > 180)
			ang_error -= 360;
		else if (ang_error < -180)
			ang_error += 360;

		// convert to radians
		ang_error *= -M_PI / 180;

	} else if (lin_error < min_error) {
		ang_error = 0; // prevent spinning
	}

	// integral values
	static double in_lin;
	static double in_ang;

	// reverse if point is behind robot
	int reverse = 1;
	if (fabs(ang_error) > M_PI_2 && (mode == ODOM || mode == ODOM_THRU)) {
		ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
		reverse = -1;
	}

	// calculate pid
	double ang_speed = pid(ang_error, &pe_ang, &in_ang, angular_pointKP,
	                       angular_pointKI, angular_pointKD);
	double lin_speed = pid(lin_error, &pe_lin, &in_lin, linear_pointKP,
	                       linear_pointKI, linear_pointKD);

	if (mode == ODOM_THRU) {
		lin_speed = chassis::maxSpeed;
	}

	lin_speed *= reverse; // apply reversal

	// add speeds together
	double left_speed = lin_speed - ang_speed;
	double right_speed = lin_speed + ang_speed;

	// speed scaling
	if (left_speed > chassis::maxSpeed) {
		double diff = left_speed - chassis::maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	} else if (left_speed < -chassis::maxSpeed) {
		double diff = left_speed + chassis::maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	}

	if (right_speed > chassis::maxSpeed) {
		double diff = right_speed - chassis::maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	} else if (right_speed < -chassis::maxSpeed) {
		double diff = right_speed + chassis::maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	}
	return {left_speed, right_speed};
}

void init(bool debug, double linearKP, double linearKI, double linearKD,
          double angularKP, double angularKI, double angularKD,
          double linear_pointKP, double linear_pointKI, double linear_pointKD,
          double angular_pointKP, double angular_pointKI,
          double angular_pointKD, double arcKP, double difKP, double min_error,
          double difMax) {

	pid::debug = debug;
	pid::linearKP = linearKP;
	pid::linearKI = linearKI;
	pid::linearKD = linearKD;
	pid::angularKP = angularKP;
	pid::angularKI = angularKI;
	pid::angularKD = angularKD;
	pid::linear_pointKP = linear_pointKP;
	pid::linear_pointKI = linear_pointKI;
	pid::linear_pointKD = linear_pointKD;
	pid::angular_pointKP = angular_pointKP;
	pid::angular_pointKI = angular_pointKI;
	pid::angular_pointKD = angular_pointKD;
	pid::arcKP = arcKP;
	pid::difKP = difKP;
	pid::min_error = min_error;
	pid::difMax = difMax;
}

} // namespace arms::pid

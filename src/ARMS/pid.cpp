#include "ARMS/api.h"
#include "api.h"

namespace arms::pid {

int mode = DISABLE;

// pid constants
double linearKP;
double linearKD;
double angularKP;
double angularKD;

double defaultLinearKP;
double defaultLinearKD;
double defaultAngularKP;
double defaultAngularKD;

double arcKP;
double difKP;

int direction;
bool thru;

// pid targets
double linearTarget = 0;
double angularTarget = 0;
std::array<double, 2> pointTarget{0, 0};

double pid(double error, double* pe, double kp, double kd) {
	double derivative = error - *pe;
	double speed = error * kp + derivative * kd;

	*pe = error;

	return speed;
}

double pid(double target, double sv, double* pe, double kp, double kd) {
	double error = target - sv;
	return pid(error, pe, kp, kd);
}

std::array<double, 2> linear() {
	static double pe = 0; // previous error

	// apply defaults
	if (linearKP == 0)
		linearKP = defaultLinearKP;
	if (linearKD == 0)
		linearKD = defaultLinearKP;

	double sv = chassis::distance();
	double speed = pid(linearTarget, sv, &pe, linearKP, linearKD);
	speed = chassis::limitSpeed(speed, chassis::maxSpeed);

	// difference PID
	std::array<double, 2> encoders = chassis::getEncoders();
	double dif = (encoders[0] - encoders[1]) * difKP;

	// disable PID for thru movement
	if (thru)
		speed = chassis::maxSpeed;

	return {speed - dif, speed + dif};
}

std::array<double, 2> angular() {
	static double pe = 0; // previous error

	// apply defaults
	if (angularKP == 0)
		angularKP = defaultAngularKP;
	if (angularKD == 0)
		angularKD = defaultAngularKP;

	double sv = chassis::angle();
	double speed = pid(angularTarget, sv, &pe, angularKP, angularKD);
	return {speed, -speed}; // clockwise positive
}

std::array<double, 2> odom() {
	// previous sensor values
	static double pe_lin = 0;
	static double pe_ang = 0;

	// get current error
	double lin_error = odom::getDistanceError(pointTarget);
	double ang_error = odom::getAngleError(pointTarget);

	// calculate linear
	if (linearKP == 0)
		linearKP = defaultLinearKP;
	if (linearKD == 0)
		linearKD = defaultLinearKP;
	double lin_speed = pid(lin_error, &pe_lin, linearKP, linearKD);

	// calculate angular
	if (angularKP == 0)
		angularKP = defaultAngularKP;
	if (angularKD == 0)
		angularKD = defaultAngularKP;
	double ang_speed = pid(ang_error, &pe_ang, angularKP, angularKD);

	// apply direction
	if (direction == 3 || (direction == 1 && fabs(ang_error) > M_PI_2)) {
		ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
		lin_speed = -lin_speed;
	}

	// limit speeds
	lin_speed = chassis::limitSpeed(lin_speed, chassis::maxSpeed);
	ang_speed = chassis::limitSpeed(ang_speed, chassis::maxSpeed);

	// disable PID for thru movement
	if (thru)
		lin_speed = chassis::maxSpeed;

	// add speeds together
	double left_speed = lin_speed - ang_speed;
	double right_speed = lin_speed + ang_speed;

	return {left_speed, right_speed};
}

void init(double linearKP, double linearKD, double angularKP, double angularKD,
          double arcKP, double difKP) {

	pid::defaultLinearKP = linearKP;
	pid::defaultLinearKD = linearKD;
	pid::defaultAngularKP = angularKP;
	pid::defaultAngularKD = angularKD;
	pid::arcKP = arcKP;
	pid::difKP = difKP;
}

} // namespace arms::pid

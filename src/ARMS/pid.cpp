#include "ARMS/lib.h"
#include "api.h"

namespace arms::pid {

int mode = DISABLE;

// pid constants
double linearKP;
double angularKP;
double linearKI;
double linearKD;
double angularKI;
double angularKD;

// kp defaults
double defaultLinearKP;
double defaultAngularKP;

double difKP;
double feedforward;

int direction;
bool thru;

// pid targets
double linearTarget = 0;
double angularTarget = 0;
std::array<double, 2> pointTarget{0, 0};

double pid(double error, double* pe, double* in, double kp, double ki,
           double kd) {

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

	if (linearKP == -1)
		linearKP = defaultLinearKP;

	double sv = chassis::distance();
	double speed = pid(linearTarget, sv, &pe, &in, linearKP, linearKI, linearKD);
	speed = chassis::limitSpeed(speed, chassis::maxSpeed);

	if (abs(speed) < feedforward)
		speed = feedforward * speed / fabs(speed);

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
	static double in = 0; // integral

	if (angularKP == -1)
		angularKP = defaultAngularKP;

	double speed = pid(angularTarget, &pe, &in, angularKP, angularKI, angularKD);
	return {speed, -speed}; // clockwise positive
}

std::array<double, 2> odom() {
	// previous sensor values
	static double pe_lin = 0;
	static double pe_ang = 0;

	// integral values
	static double in_lin;
	static double in_ang;

	// get current error
	double lin_error = odom::getDistanceError(pointTarget);
	double ang_error = odom::getAngleError(pointTarget);

	// check for default kp
	if (linearKP == -1)
		linearKP = defaultLinearKP;
	if (angularKP == -1)
		angularKP = defaultAngularKP;

	double lin_speed =
	    pid(lin_error, &pe_lin, &in_lin, linearKP, linearKI, linearKD);

	double ang_speed =
	    pid(ang_error, &pe_ang, &in_ang, angularKP, angularKI, angularKD);

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

void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double difKP,
          double feedforward) {

	pid::defaultLinearKP = linearKP;
	pid::linearKI = linearKI;
	pid::linearKD = linearKD;
	pid::defaultAngularKP = angularKP;
	pid::angularKI = angularKI;
	pid::angularKD = angularKD;
	pid::difKP = difKP;
	pid::feedforward = feedforward;
}

} // namespace arms::pid

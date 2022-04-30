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
double trackingKP;
double minError;

// integral
double in_lin;
double in_ang;

// kp defaults
double defaultLinearKP;
double defaultAngularKP;
double defaultTrackingKP;

// flags
bool reverse;
bool thru;

// pid targets
double angularTarget = 0;
Point pointTarget{0, 0};

double pid(double error, double* pe, double* in, double kp, double ki,
           double kd) {

	double derivative = error - *pe;
	if ((*pe > 0 && error < 0) || (*pe < 0 && error > 0))
		*in = 0; // remove integral at zero error
	double speed = error * kp + *in * ki + derivative * kd;

	// only let integral wind up if near the target
	if (fabs(error) < 15) {
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

std::array<double, 2> translational() {
	// previous sensor values
	static double pe_lin = 0;
	static double pe_ang = 0;

	// get current error
	double lin_error = odom::getDistanceError(pointTarget);
	double ang_error = odom::getAngleError(pointTarget);

	// check for default kp
	if (linearKP == -1)
		linearKP = defaultLinearKP;
	if (trackingKP == -1)
		trackingKP = defaultTrackingKP;

	// calculate speeds with PID
	double lin_speed =
	    pid(lin_error, &pe_lin, &in_lin, linearKP, linearKI, linearKD);

	double ang_speed =
	    pid(ang_error, &pe_ang, &in_ang, trackingKP, 0, 0);


	// disable PID for thru movement
	if (thru)
		lin_speed = chassis::maxSpeed;

	// cap linear speed
	if (lin_speed > 100)
		lin_speed = 100;

	// overturn
	double overturn = fabs(ang_speed) + lin_speed - 100;
	if(overturn > 0)
		lin_speed -= overturn;

	// apply direction
	if (reverse)
		lin_speed = -lin_speed;

	// prevent spinning around the point
	if(lin_error < minError){
		ang_speed = 0;
		lin_speed *= cos(ang_error);
		if (fabs(ang_error) > M_PI_2) {
			ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
			lin_speed = -lin_speed;
		}
	}

	// add speeds together
	double left_speed = lin_speed - ang_speed;
	double right_speed = lin_speed + ang_speed;

	return {left_speed, right_speed};
}

std::array<double, 2> angular() {
	static double pe = 0; // previous error

	if (angularKP == -1)
		angularKP = defaultAngularKP;

	double sv = odom::getHeading();
	double speed =
	    pid(angularTarget, sv, &pe, &in_ang, angularKP, angularKI, angularKD);
	return {-speed, speed}; // clockwise positive
}

void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double trackingKP, double minError) {

	pid::defaultLinearKP = linearKP;
	pid::linearKI = linearKI;
	pid::linearKD = linearKD;
	pid::defaultAngularKP = angularKP;
	pid::angularKI = angularKI;
	pid::angularKD = angularKD;
	pid::defaultTrackingKP = trackingKP;
	pid::minError = minError;
}

} // namespace arms::pid

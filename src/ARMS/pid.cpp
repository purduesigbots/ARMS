#include "ARMS/api.h"
#include "api.h"
#include "chassis.h"

namespace arms::pid {

int mode = DISABLE;

// constants
double linearKP;
double angularKP;
double linearKI;
double linearKD;
double angularKI;
double angularKD;
double trackingKP;
double trackingKD;
double minError;
double leadPct;

// integral
double in_lin;
double in_ang;

// kp defaults
double defaultLinearKP;
double defaultAngularKP;
double defaultTrackingKP;
double defaultTrackingKD;

// flags
bool reverse;
bool thru;

// pid targets
double angularTarget = 0;
Point pointTarget{0, 0};

bool canReverse;

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

	// an angular target > 360 indicates no desired final pose angle
	bool noPose = (angularTarget > 360);

	// Determine target point
	Point carrotPoint; // chasing the carrot
	if (noPose) {
		// point movement
		carrotPoint = pointTarget;
	} else {
		// pose movement
		double h = odom::getDistanceError(pointTarget);
		double at = angularTarget * M_PI / 180.0;
		carrotPoint = {pointTarget.x - h * cos(at) * leadPct,
		               pointTarget.y - h * sin(at) * leadPct};
	}

	// get current error
	double lin_error = odom::getDistanceError(pointTarget);
	double ang_error = odom::getAngleError(carrotPoint);

	// check for default kp
	if (linearKP == -1)
		linearKP = defaultLinearKP;
	if (trackingKP == -1)
		trackingKP = defaultTrackingKP;
	if (trackingKD == -1)
		trackingKD = defaultTrackingKD;


	// calculate linear speed
	double lin_speed;
	if (thru)
		lin_speed = chassis::maxSpeed; // disable PID for thru movement
	else
		lin_speed = pid(lin_error, &pe_lin, &in_lin, linearKP, linearKI, linearKD);

	// cap linear speed
	if (lin_speed > chassis::maxSpeed)
		lin_speed = chassis::maxSpeed;

	if (lin_speed < chassis::min_linear_speed)
		lin_speed = chassis::min_linear_speed;

	// apply direction
	if (reverse)
		lin_speed = -lin_speed;

	// calculate angular speed
	double ang_speed;
	if (lin_error < minError) {
		canReverse = true;

		if (noPose) {
			ang_speed =
			    0; // disable turning when close to the point to prevent spinning
		} else {
			// turn to face the finale pose angle if executing a pose movement
			double poseError = (angularTarget * M_PI / 180) - odom::getHeading(true);
			while (fabs(poseError) > M_PI)
				poseError -= 2 * M_PI * poseError / fabs(poseError);
			ang_speed = pid(poseError, &pe_ang, &in_ang, trackingKP, 0, trackingKD);

			if (fabs(ang_speed) < chassis::min_angular_speed) {
				ang_speed =
				    (std::signbit(ang_speed) ? -1 : 1) * chassis::min_angular_speed;
			}
		}

		// reduce the linear speed if the bot is tangent to the target
		lin_speed *= cos(ang_error);

	} else {
		// reverse on overshoot
		if (fabs(ang_error) > M_PI_2 && canReverse) {
			ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
			lin_speed = -lin_speed;
		}

		ang_speed = pid(ang_error, &pe_ang, &in_ang, trackingKP, 0, trackingKD);
	}

	// overturn
	double overturn = fabs(ang_speed) + fabs(lin_speed) - chassis::maxSpeed;
	if (overturn > 0)
		lin_speed -= lin_speed > 0 ? overturn : -overturn;

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
          double angularKI, double angularKD, double trackingKP, double trackingKD,
          double minError, double leadPct) {

	pid::defaultLinearKP = linearKP;
	pid::linearKI = linearKI;
	pid::linearKD = linearKD;
	pid::defaultAngularKP = angularKP;
	pid::angularKI = angularKI;
	pid::angularKD = angularKD;
	pid::defaultTrackingKP = trackingKP;
	pid::defaultTrackingKD = trackingKD;
	pid::minError = minError;
	pid::leadPct = leadPct;
}

} // namespace arms::pid

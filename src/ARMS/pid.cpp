#include "ARMS/api.h"
#include "api.h"
#include "chassis.h"
#include "pros/rtos.hpp"
namespace arms::pid {

int mode = DISABLE;

// mutex protection
pros::Mutex pidMutex;

// constants
std::atomic<double> linearKP;
std::atomic<double> angularKP;
std::atomic<double> linearKI;
std::atomic<double> linearKD;
std::atomic<double> angularKI;
std::atomic<double> angularKD;
std::atomic<double> trackingKP;
std::atomic<double> minError;
std::atomic<double> leadPct;

// integral
std::atomic<double> in_lin;
std::atomic<double> in_ang;

// kp defaults
std::atomic<double> defaultLinearKP;
std::atomic<double> defaultAngularKP;
std::atomic<double> defaultTrackingKP;

// flags
bool reverse;
bool thru;

// pid targets
std::atomic<double> angularTarget = 0;
Point pointTarget{0, 0};

bool canReverse;

double pid(double error, double* pe, double* in, double kp, double ki,
           double kd) {
	const std::lock_guard<pros::Mutex> lock(pidMutex);
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
	pidMutex.take();
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
		linearKP.store(defaultLinearKP);
	if (trackingKP == -1)
		trackingKP.store(defaultTrackingKP);

	// calculate linear speed
	double lin_speed;
	if (thru) {
		lin_speed = chassis::maxSpeed; // disable PID for thru movement
	}
	else {
		double new_pe_lin, new_in_lin;
		pidMutex.give();
		lin_speed = pid(lin_error, &new_pe_lin, &new_in_lin, linearKP, linearKI, linearKD);
		pidMutex.take();
		// new values due to std::atomic not allowing ptrs
		pe_lin = new_pe_lin;
		in_lin = new_in_lin;
	}
	// cap linear speed
	if (lin_speed > chassis::maxSpeed)
		lin_speed = chassis::maxSpeed;

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
			
			double new_pe_ang, new_in_ang;
			pidMutex.give();
			ang_speed = pid(poseError, &new_pe_ang, &new_in_ang, trackingKP, 0, 0);
			pidMutex.take();
			// new values due to std::atomic not allowing ptrs
			pe_ang = new_pe_ang;
			in_ang = new_in_ang;
		}

		// reduce the linear speed if the bot is tangent to the target
		lin_speed *= cos(ang_error);

	} else {
		// reverse on overshoot
		if (fabs(ang_error) > M_PI_2 && canReverse) {
			ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
			lin_speed = -lin_speed;
		}
		double new_pe_ang, new_in_ang;
		pidMutex.give();
		ang_speed = pid(ang_error, &new_pe_ang, &new_in_ang, trackingKP, 0, 0);
		pidMutex.take();
		// new values due to std::atomic not allowing ptrs
		pe_ang = new_pe_ang;
		in_ang = new_in_ang;
	}

	// overturn
	double overturn = fabs(ang_speed) + fabs(lin_speed) - chassis::maxSpeed;
	if (overturn > 0)
		lin_speed -= lin_speed > 0 ? overturn : -overturn;

	// add speeds together
	double left_speed = lin_speed - ang_speed;
	double right_speed = lin_speed + ang_speed;
	
	pidMutex.give();

	return {left_speed, right_speed};
}

std::array<double, 2> angular() {
	pidMutex.take();
	static double pe = 0; // previous error

	if (angularKP == -1)
		angularKP.store(defaultAngularKP);

	double sv = odom::getHeading();

	pidMutex.give();
	double new_pe, new_in;
	double speed =
	    pid(angularTarget, sv, &new_pe, &new_in, angularKP, angularKI, angularKD);
	pe = new_pe;
	in_ang = new_in;

	return {-speed, speed}; // clockwise positive
}

void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double trackingKP,
          double minError, double leadPct) {
	// inherently thread safe as theses are all atomics
	pid::defaultLinearKP = linearKP;
	pid::linearKI = linearKI;
	pid::linearKD = linearKD;
	pid::defaultAngularKP = angularKP;
	pid::angularKI = angularKI;
	pid::angularKD = angularKD;
	pid::defaultTrackingKP = trackingKP;
	pid::minError = minError;
	pid::leadPct = leadPct;
}

} // namespace arms::pid

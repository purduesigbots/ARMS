#include "ARMS/pid.h"
#include "ARMS/chassis.h"
#include "ARMS/odom.h"

namespace arms::pid {

int PID::getMode() {
	return mode;
}

void PID::setMode(int newMode) {
	mode = newMode;
}

std::array<double, 2> PID::getPointTarget() {
	return pointTarget;
}

void PID::setPointTarget(std::array<double, 2> newTarget) {
	pointTarget = newTarget;
}

double PID::getLinearTarget() {
	return linearTarget;
}

void PID::setLinearTarget(double newTarget) {
	linearTarget = newTarget;
}

double PID::getAngularTarget() {
	return angularTarget;
}

void PID::setAngularTarget(double newTarget) {
	angularTarget = newTarget;
}

double PID::getVectorAngle() {
	return vectorAngle;
}

void PID::setVectorAngle(double newAngle) {
	vectorAngle = newAngle;
}

double PID::getDifKP() {
	return difKP;
}

double PID::getArcKP() {
	return arcKP;
}

double PID::pid(double error, double* pe, double* in, double kp, double ki,
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

double PID::pid(double target, double sv, double* pe, double* in, double kp,
                double ki, double kd) {
	double error = target - sv;
	return pid(error, pe, in, kp, ki, kd);
}

std::array<double, 2> PID::linear(double sv_x, double sv_y, double maxSpeed,
                                  double difference) {
	static double pe = 0; // previous error
	static double in = 0; // integral

	// calculate total displacement using pythagorean theorem
	double sv;
	if (vectorAngle != 0)
		sv = sqrt(pow(sv_x, 2) + pow(sv_y, 2));
	else
		sv = sv_x; // just use the x value for non-holonomic movements

	double speed = pid(linearTarget, sv, &pe, &in, linearKP, linearKI, linearKD);

	if (speed > maxSpeed)
		speed = maxSpeed;
	if (speed < -maxSpeed)
		speed = -maxSpeed;

	// difference PID
	double dif = difference * difKP;

	// prevent oscillations near target
	if (dif > speed && speed > 0)
		dif = speed;
	else if (dif < -speed && speed < 0)
		dif = -speed;

	return {speed -= dif, speed += dif};
}

std::array<double, 2> PID::angular(double angle) {
	static double pe = 0; // previous error
	static double in = 0; // integral
	double sv = angle;
	double speed =
	    pid(angularTarget, sv, &pe, &in, angularKP, angularKI, angularKD);
	return {speed, -speed}; // clockwise positive
}

std::array<double, 2> PID::odom(double maxSpeed, double g_x, double g_y,
                                double heading, double heading_degrees) {
	// previous sensor values
	static double psv_left = 0;
	static double psv_right = 0;

	double y = pointTarget[0];
	double x = pointTarget[1];

	y -= g_y;
	x -= g_x;

	double lin_error = sqrt(x * x + y * y); // linear

	double delta_theta = heading - atan2(x, y);

	while (fabs(delta_theta) > M_PI) {
		delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
	}

	double ang_error = delta_theta; // angular

	// if holonomic, angular error is relative to field, not to point
	if (mode == ODOM_HOLO || mode == ODOM_HOLO_THRU) {
		vectorAngle = ang_error;
		ang_error = angularTarget - ((int)heading_degrees % 360);

		// make sure all turns take most efficient route
		if (ang_error > 180)
			ang_error -= 360;
		else if (ang_error < -180)
			ang_error += 360;

		// convert to radians
		ang_error = ang_error * M_PI / 180;

	} else if (lin_error < min_error) {
		ang_error = 0; // prevent spinning
	}

	// previous error values
	static double pe_lin = lin_error;
	static double pe_ang = ang_error;

	// integral values
	static double in_lin;
	static double in_ang;

	// reverse if point is behind robot
	int reverse = 1;
	if (fabs(ang_error) > M_PI_2 && mode == ODOM) {
		ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
		reverse = -1;
	}

	// calculate pid
	double ang_speed = pid(ang_error, &pe_ang, &in_ang, angular_pointKP,
	                       angular_pointKI, angular_pointKD);
	double lin_speed = pid(lin_error, &pe_lin, &in_lin, linear_pointKP,
	                       linear_pointKI, linear_pointKD);

	// store previous previos error
	pe_lin = lin_error;
	pe_ang = ang_error;

	lin_speed *= reverse; // apply reversal

	// add speeds together
	double left_speed = lin_speed + ang_speed;
	double right_speed = lin_speed - ang_speed;

	// speed scaling
	if (left_speed > maxSpeed) {
		double diff = left_speed - maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	} else if (left_speed < -maxSpeed) {
		double diff = left_speed + maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	}

	if (right_speed > maxSpeed) {
		double diff = right_speed - maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	} else if (right_speed < -maxSpeed) {
		double diff = right_speed + maxSpeed;
		left_speed -= diff;
		right_speed -= diff;
	}

	return {left_speed, right_speed};
}

PID::PID(bool debug, double linearKP, double linearKI, double linearKD,
         double angularKP, double angularKI, double angularKD,
         double linear_pointKP, double linear_pointKI, double linear_pointKD,
         double angular_pointKP, double angular_pointKI, double angular_pointKD,
         double arcKP, double difKP, double min_error) {

	this->debug = debug;
	this->linearKP = linearKP;
	this->linearKI = linearKI;
	this->linearKD = linearKD;
	this->angularKP = angularKP;
	this->angularKI = angularKI;
	this->angularKD = angularKD;
	this->linear_pointKP = linear_pointKP;
	this->linear_pointKI = linear_pointKI;
	this->linear_pointKD = linear_pointKD;
	this->angular_pointKP = angular_pointKP;
	this->angular_pointKI = angular_pointKI;
	this->angular_pointKD = angular_pointKD;
	this->arcKP = arcKP;
	this->difKP = difKP;
	this->min_error = min_error;
}

} // namespace arms::pid

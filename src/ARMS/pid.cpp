#include "ARMS/pid.h"
#include "ARMS/chassis.h"
#include "ARMS/odom.h"

namespace pid {

int mode = DISABLE;

// pid constants
double linearKP;
double linearKD;
double angularKP;
double angularKD;
double linear_pointKP;
double linear_pointKD;
double angular_pointKP;
double angular_pointKD;
double arcKP;
double difKP;

// pid targets
double linearTarget = 0;
double angularTarget = 0;
double vectorAngle = 0;
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

	// get position in the x and y directions
	double sv_x = chassis::position();
	double sv_y = chassis::position(true);

	// calculate total displacement using pythagorean theorem
	double sv;
	if (vectorAngle != 0)
		sv = sqrt(pow(sv_x, 2) + pow(sv_y, 2));
	else
		sv = sv_x; // just use the x value for non-holonomic movements

	double speed = pid(linearTarget, sv, &pe, linearKP, linearKD);

	// difference PID
	double dif = chassis::difference() * difKP;
	return {speed -= dif, speed += dif};
}

double angular() {
	static double pe = 0; // previous error
	double sv = chassis::angle();
	double speed = pid(angularTarget, sv, &pe, angularKP, angularKD);
	return speed;
}

std::array<double, 2> gtp() {
	// previous sensor values
	static double psv_left = 0;
	static double psv_right = 0;

	double lin_error = odom::getDistanceError(pointTarget); // linear
	double ang_error = odom::getAngleError(pointTarget);    // angular

	// previous error values
	static double pe_lin = lin_error;
	static double pe_ang = ang_error;

	// reverse if point is behind robot
	int reverse = 1;
	if (fabs(ang_error) > M_PI_2) {
		ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
		reverse = -1;
	}

	// calculate pid
	double lin_speed = pid(lin_error, &pe_lin, linear_pointKP, linear_pointKD);
	double ang_speed = pid(ang_error, &pe_ang, linear_pointKP, linear_pointKD);

	// store previous previos error
	pe_lin = lin_error;
	pe_ang = ang_error;

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

void init(double linearKP, double linearKD, double angularKP, double angularKD,
          double linear_pointKP, double linear_pointKD, double angular_pointKP,
          double angular_pointKD, double arcKP, double difKP) {

	pid::linearKP = linearKP;
	pid::linearKD = linearKD;
	pid::angularKP = angularKP;
	pid::angularKD = angularKD;
	pid::linear_pointKP = linear_pointKP;
	pid::linear_pointKD = linear_pointKD;
	pid::angular_pointKP = angular_pointKP;
	pid::angular_pointKD = angular_pointKD;
	pid::arcKP = arcKP;
	pid::difKP = difKP;
}

} // namespace pid

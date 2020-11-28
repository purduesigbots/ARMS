#include "chassis.h"

namespace pid {

// pid constants
double linearKP;
double linearKD;
double angularKP;
double angularKD;
double arcKP;
double difKP;

// pid targets
double linearTarget = 0;
double angularTarget = 0;
double vectorAngle = 0;

double pid(double target, double sv, double* psv, double kp, double kd) {
	double error = target - sv;
	double derivative = error - *psv;
	*psv = error;
	double speed = error * linearKP + derivative * linearKD;
	return speed;
}

double linear(bool rightSide = false) {
	static double psv = 0;

	// get position in the x and y directions
	double sv_x = chassis::position();
	double sv_y = chassis::position(true);

	// calculate total displacement using pythagorean theorem
	double sv;
	if (vectorAngle != 0)
		sv = sqrt(pow(sv_x, 2) + pow(sv_y, 2));
	else
		sv = sv_x; // just use the x value for non-holonomic movements

	double speed = pid(linearTarget, sv, &psv, linearKP, linearKD);

	// difference PID
	double dif = chassis::difference() * difKP;
	if (rightSide)
		speed += dif;
	else
		speed -= dif;

	return speed;
}

double angular() {
	static double psv = 0;
	double sv = chassis::angle();
	double speed = pid(angularTarget, sv, &psv, angularKP, angularKD);
	return speed;
}

void init(double linearKP, double linearKD, double angularKP, double angularKD,
          double arcKP, double difKP) {

	pid::linearKP = linearKP;
	pid::linearKD = linearKD;
	pid::angularKP = angularKP;
	pid::angularKD = angularKD;
	pid::arcKP = arcKP;
	pid::difKP = difKP;
}

} // namespace pid

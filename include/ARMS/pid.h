#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include "ARMS/config.h"
#include <array>

namespace arms::pid {

// pid mode enums
#define ODOM_HOLO_THRU 5
#define ODOM_HOLO 4
#define ODOM 3
#define ANGULAR 2
#define LINEAR 1
#define DISABLE 0

class PID {
private:
	int mode;
	bool debug;

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

	double linearTarget;
	double angularTarget;
	double vectorAngle;
	std::array<double, 2> pointTarget;

	double arcKP; // needs to be exposed since arcs have not been
	              // integrated into new PID format

	double difKP; // needs to be exposed for use with chassis::fast

	double min_error;

public:
	int getMode();
	void setMode(int newMode);

	std::array<double, 2> getPointTarget();
	void setPointTarget(std::array<double, 2> newTarget);
	double getLinearTarget();
	void setLinearTarget(double newTarget);
	double getAngularTarget();
	void setAngularTarget(double newTarget);
	double getVectorAngle();
	void setVectorAngle(double newAngle);

	double getDifKP();
	double getArcKP();

	double pid(double error, double* pe, double* in, double kp, double ki,
	           double kd);
	double pid(double target, double sv, double* pe, double* in, double kp,
	           double ki, double kd);
	std::array<double, 2> linear(double sv_x, double sv_y, double maxSpeed,
	                             double difference);
	std::array<double, 2> angular(double angle);
	std::array<double, 2> odom(double maxSpeed, double g_x, double g_y,
	                           double heading, double heading_degrees);

	PID(bool debug = PID_DEBUG, double linearKP = LINEAR_KP,
	    double linearKI = LINEAR_KI, double linearKD = LINEAR_KD,
	    double angularKP = ANGULAR_KP, double angularKI = ANGULAR_KI,
	    double angularKD = ANGULAR_KD, double linear_pointKP = LINEAR_POINT_KP,
	    double linear_pointKI = LINEAR_POINT_KI,
	    double linear_pointKD = LINEAR_POINT_KD,
	    double angular_pointKP = ANGULAR_POINT_KP,
	    double angular_pointKI = ANGULAR_POINT_KI,
	    double angular_pointKD = ANGULAR_POINT_KD, double arcKP = ARC_KP,
	    double difKP = DIF_KP, double min_error = MIN_ERROR);
};

} // namespace arms::pid

#endif

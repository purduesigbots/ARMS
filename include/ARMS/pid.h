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
protected:
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

	PID(bool debug = false, double linearKP = 0, double linearKI = 0,
	    double linearKD = 0, double angularKP = 0, double angularKI = 0,
	    double angularKD = 0, double linear_pointKP = 0,
	    double linear_pointKI = 0, double linear_pointKD = 0,
	    double angular_pointKP = 0, double angular_pointKI = 0,
	    double angular_pointKD = 0, double arcKP = 0, double difKP = 0,
	    double min_error = 0);
};

class PIDBuilder {
protected:
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
	double arcKP;
	double difKP;
	double min_error;

public:
	PIDBuilder();

	PIDBuilder& withLinearPID(double kP, double kI, double kD);
	PIDBuilder& withAngularPID(double kP, double kI, double kD);
	PIDBuilder& withLinearPointPID(double kP, double kI, double kD);
	PIDBuilder& withAngularPointPID(double kP, double kI, double kD);
	PIDBuilder& withArcKP(double kP);
	PIDBuilder& withDifKP(double kP);
	PIDBuilder& withMinError(double m);

	PID build();
};

} // namespace arms::pid

#endif

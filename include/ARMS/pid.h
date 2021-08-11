#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include "ARMS/config.h"
#include <array>

namespace arms::pid {

/// Enum used to define PID mode
enum class PIDMode {
	ODOM_HOLO_THRU,
	ODOM_HOLO,
	ODOM,
	ANGULAR,
	LINEAR,
	DISABLE
};

class PID {
protected:
	PIDMode mode;
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
	/// Gets the PID mode
	PIDMode getMode();

	/// Sets the PID mode
	void setMode(PIDMode newMode);

	/// Gets the point target
	std::array<double, 2> getPointTarget();

	/// Sets the point target
	void setPointTarget(std::array<double, 2> newTarget);

	/// Gets the linear target
	double getLinearTarget();

	/// Sets the linear target
	void setLinearTarget(double newTarget);

	/// Gets the angular target
	double getAngularTarget();

	/// Sets the angular target
	void setAngularTarget(double newTarget);

	/// Gets the vector angle
	double getVectorAngle();

	/// Sets the vector angle
	void setVectorAngle(double newAngle);

	/// Gets the difference kP value
	double getDifKP();

	/// Gets the arc kP value
	double getArcKP();

	/// Returns speed using the given error and PID values
	double pid(double error, double* pe, double* in, double kp, double ki,
	           double kd);

	/// Returns speed using the given target, setpoint, and PID values
	double pid(double target, double sv, double* pe, double* in, double kp,
	           double ki, double kd);

	/// Returns chassis motor speeds using PID for linear motions
	std::array<double, 2> linear(double sv_x, double sv_y, double maxSpeed,
	                             double difference);

	/// Returns chassis motor speeds using PID for angular motions
	std::array<double, 2> angular(double angle);

	/// Returns chassis motor speeds using PID for odometry motions
	std::array<double, 2> odom(double maxSpeed, double g_x, double g_y,
	                           double heading, double heading_degrees);

	/// A class that uses PID loops to control chassis movement
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
	/// A builder to make PID objects
	PIDBuilder();

	/// Sets the linear PID values
	PIDBuilder& withLinearPID(double kP, double kI, double kD);

	/// Sets the angular PID values
	PIDBuilder& withAngularPID(double kP, double kI, double kD);

	/// Sets the linear point PID values for odometry
	PIDBuilder& withLinearPointPID(double kP, double kI, double kD);

	/// Sets the angular point PID values for odometry
	PIDBuilder& withAngularPointPID(double kP, double kI, double kD);

	/// Sets the arc kP value
	PIDBuilder& withArcKP(double kP);

	/// Sets the difference kP value
	PIDBuilder& withDifKP(double kP);

	/// Sets the minimum error value
	PIDBuilder& withMinError(double m);

	/// Builds a PID object
	PID build();
};

} // namespace arms::pid

#endif

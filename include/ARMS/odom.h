#ifndef _ARMS_ODOM_H_
#define _ARMS_ODOM_H_

#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include <array>

namespace arms::odom {

class OdomChassis : public chassis::Chassis {
protected:
	bool debug;
	double left_right_distance;
	double middle_distance;
	double left_right_tpi;
	double middle_tpi;
	double exit_error;
	bool holonomic;

	double prev_heading;
	double prev_right_pos;
	double prev_left_pos;
	double prev_middle_pos;

public:
	void resetOdom(std::array<double, 2> point = {0, 0});

	void resetOdom(std::array<double, 2> point, double angle);

	double getAngleError(std::array<double, 2> point);

	double getDistanceError(std::array<double, 2> point);

	void moveAsync(std::array<double, 2> point, double max = 80);

	void holoAsync(std::array<double, 2> point, double angle, double max = 80,
	               double turnMax = 50);

	void move(std::array<double, 2> point, double max = 80);

	void moveThru(std::array<double, 2> point, double max = 80);

	void holo(std::array<double, 2> point, double angle, double max = 80,
	          double turnMax = 50);

	void holoThru(std::array<double, 2> point, double angle, double max = 80,
	              double turnMax = 50);

	std::array<double, 2>
	findIntersectionPoint(std::vector<std::array<double, 2>> path, double radius);

	void followPath(std::vector<std::array<double, 2>> path);

	void startTask();

	OdomChassis(
	    std::initializer_list<okapi::Motor> leftMotorsList = {LEFT_MOTORS},
	    std::initializer_list<okapi::Motor> rightMotorsList = {RIGHT_MOTORS},
	    int gearset = GEARSET, double distance_constant = DISTANCE_CONSTANT,
	    double degree_constant = DEGREE_CONSTANT, int settle_time = SETTLE_TIME,
	    double settle_threshold_linear = SETTLE_THRESHOLD_LINEAR,
	    double settle_threshold_angular = SETTLE_THRESHOLD_ANGULAR,
	    double accel_step = ACCEL_STEP, double arc_step = ARC_STEP,
	    int imuPort = IMU_PORT,
	    std::tuple<int, int, int> encoderPorts = {ENCODER_PORTS},
	    int expanderPort = EXPANDER_PORT,
	    int joystick_threshold = JOYSTICK_THRESHOLD, pid::PID pid = pid::PID(),
	    bool debug = ODOM_DEBUG, double left_right_distance = LEFT_RIGHT_DISTANCE,
	    double middle_distance = MIDDLE_DISTANCE,
	    double left_right_tpi = LEFT_RIGHT_TPI, double middle_tpi = MIDDLE_TPI,
	    bool holonomic = HOLONOMIC, double exit_error = EXIT_ERROR);
};

} // namespace arms::odom

#endif

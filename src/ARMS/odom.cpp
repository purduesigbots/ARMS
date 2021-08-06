#include "ARMS/chassis.h"

using namespace pros;

namespace arms::chassis {

void OdomChassis::startTask() {

	Task chassis_task([this](void) -> int {
		global_x = 0;
		global_y = 0;

		while (true) {
			double left_pos;
			double right_pos;
			double middle_pos;

			// get positions of each encoder
			if (leftEncoder) {
				left_pos = leftEncoder->get_value();
				right_pos = rightEncoder->get_value();
			} else if (holonomic) {
				left_pos = backLeft->getPosition();
				right_pos = frontRight->getPosition();
				middle_pos = backRight->getPosition();
			} else {
				left_pos = leftMotors->getPosition();
				right_pos = rightMotors->getPosition();
			}

			if (middleEncoder)
				middle_pos = middleEncoder->get_value();

			// calculate change in each encoder
			double delta_left = (left_pos - prev_left_pos) / left_right_tpi;
			double delta_right = (right_pos - prev_right_pos) / left_right_tpi;
			double delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;

			// calculate new heading
			double delta_angle;
			if (imu) {
				heading_degrees = angle();
				heading = heading_degrees * M_PI / 180.0;
				delta_angle = heading - prev_heading;
			} else {
				delta_angle = (delta_left - delta_right) / (left_right_distance * 2);

				heading += delta_angle;
				heading_degrees = heading * 180.0 / M_PI;
			}

			// store previous positions
			prev_left_pos = left_pos;
			prev_right_pos = right_pos;
			prev_middle_pos = middle_pos;
			prev_heading = heading;

			// calculate local displacemente
			double local_x;
			double local_y;

			if (delta_angle) {
				double i = sin(delta_angle / 2.0) * 2.0;
				local_y = (delta_right / delta_angle + left_right_distance) * i;
				local_x = (delta_middle / delta_angle + middle_distance) * i;
			} else {
				local_y = delta_right;
				local_x = delta_middle;
			}

			double p = heading - delta_angle / 2.0; // global angle

			// account for holonomic rotation
			if (holonomic)
				p -= M_PI / 4;

			// convert to absolute displacement
			global_y += cos(p) * local_y - sin(p) * local_x;
			global_x += sin(p) * local_y + cos(p) * local_x;

			if (debug)
				printf("%.2f, %.2f, %.2f \n", global_x, global_y, heading_degrees);

			delay(10);
		}
	});
}

void OdomChassis::resetOdom(std::array<double, 2> point) {
	global_y = point[0];
	global_x = point[1];
}

void OdomChassis::resetOdom(std::array<double, 2> point, double angle) {
	resetOdom({point[0], point[1]});
	heading = angle * M_PI / 180.0;
	prev_heading = heading;
	resetAngle(angle);
}

double OdomChassis::getAngleError(std::array<double, 2> point) {
	double y = point[0];
	double x = point[1];

	y -= global_y;
	x -= global_x;

	double delta_theta = heading - atan2(x, y);

	while (fabs(delta_theta) > M_PI) {
		delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
	}

	return delta_theta;
}

double OdomChassis::getDistanceError(std::array<double, 2> point) {
	double y = point[0];
	double x = point[1];

	y -= global_y;
	x -= global_x;
	return sqrt(x * x + y * y);
}

void OdomChassis::moveAsync(std::array<double, 2> point, double max) {
	prev_left_pos = 0;
	prev_right_pos = 0;
	prev_middle_pos = 0;

	reset();

	maxSpeed = max;
	pid.setPointTarget(point);
	pid.setMode(ODOM);
}

void OdomChassis::holoAsync(std::array<double, 2> point, double angle,
                            double max, double turnMax) {

	prev_left_pos = 0;
	prev_right_pos = 0;
	prev_middle_pos = 0;

	reset();

	maxSpeed = max;
	maxTurn = turnMax;
	pid.setPointTarget(point);
	pid.setAngularTarget(angle);
	pid.setMode(ODOM_HOLO);
}

void OdomChassis::move(std::array<double, 2> point, double max) {
	moveAsync(point, max);
	delay(450);
	waitUntilSettled();
}

void OdomChassis::moveThru(std::array<double, 2> point, double max) {
	moveAsync(point, max);
	delay(450);
	while (getDistanceError(point) > exit_error)
		delay(10);
}

void OdomChassis::holo(std::array<double, 2> point, double angle, double max,
                       double turnMax) {
	holoAsync(point, angle, max, turnMax);
	delay(450);
	waitUntilSettled();
}

void OdomChassis::holoThru(std::array<double, 2> point, double angle,
                           double max, double turnMax) {
	holoAsync(point, angle, max, turnMax);
	pid.setMode(ODOM_HOLO_THRU);
	delay(450);
	while (getDistanceError(point) > exit_error && !settled())
		delay(10);
}

OdomChassis::OdomChassis(std::initializer_list<okapi::Motor> leftMotorsList,
                         std::initializer_list<okapi::Motor> rightMotorsList,
                         int gearset, double distance_constant,
                         double degree_constant, int settle_time,
                         double settle_threshold_linear,
                         double settle_threshold_angular, double accel_step,
                         double arc_step, int imuPort,
                         std::tuple<int, int, int> encoderPorts,
                         int expanderPort, int joystick_threshold, pid::PID pid,
                         bool debug, double left_right_distance,
                         double middle_distance, double left_right_tpi,
                         double middle_tpi, bool holonomic, double exit_error)
    : chassis::Chassis(leftMotorsList, rightMotorsList, gearset,
                       distance_constant, degree_constant, settle_time,
                       settle_threshold_linear, settle_threshold_angular,
                       accel_step, arc_step, imuPort, encoderPorts,
                       expanderPort, joystick_threshold, pid) {
	this->debug = debug;
	this->left_right_distance = left_right_distance;
	this->middle_distance = middle_distance;
	this->left_right_tpi = left_right_tpi;
	this->middle_tpi = middle_tpi;
	if (leftEncoder)
		holonomic = false; // holonomic should only be used on non-encoder x-drives
	this->holonomic = holonomic;
	this->exit_error = exit_error;
	delay(1500);

	startTask();
}

Odom::Odom()
    : debug(false), left_right_distance(0), middle_distance(0),
      left_right_tpi(0), middle_tpi(0), exit_error(10), holonomic(false) {
}

Odom& Odom::withDebug(bool deb) {
	debug = deb;
	return *this;
}

Odom& Odom::withDistances(double left_right, double middle) {
	left_right_distance = left_right;
	middle_distance = middle;
	return *this;
}

Odom& Odom::withTPI(double left_right, double middle) {
	left_right_tpi = left_right;
	middle_tpi = middle;
	return *this;
}

Odom& Odom::withExitError(double err) {
	exit_error = err;
	return *this;
}

Odom& Odom::withHolonomic(bool holo) {
	holonomic = holo;
	return *this;
}

bool Odom::getDebug() {
	return debug;
}

double Odom::getLeftRightDistance() {
	return left_right_distance;
}

double Odom::getMiddleDistance() {
	return middle_distance;
}

double Odom::getLeftRightTPI() {
	return left_right_tpi;
}

double Odom::getMiddleTPI() {
	return middle_tpi;
}

double Odom::getExitError() {
	return exit_error;
}

bool Odom::getHolonomic() {
	return holonomic;
}

} // namespace arms::chassis

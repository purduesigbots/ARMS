#include "ARMS/lib.h"
#include "api.h"

#include <tuple>

namespace arms::chassis {

// chassis motors
std::shared_ptr<okapi::MotorGroup> leftMotors;
std::shared_ptr<okapi::MotorGroup> rightMotors;

// distance constants
double distance_constant; // ticks per inch
double degree_constant;   // ticks per degree

// slew control (autonomous only)
double slew_step; // smaller number = more slew

// default exit error
double linear_exit_error;
double angular_exit_error;

// chassis variables
double maxSpeed = 100;
double leftPrev = 0;
double rightPrev = 0;
double leftDriveSpeed = 0;
double rightDriveSpeed = 0;
Point virtualPosition;

/**************************************************/
// motor control
void motorMove(std::shared_ptr<okapi::MotorGroup> motor, double speed,
               bool velocity) {
	if (velocity)
		motor->moveVelocity(speed * (double)motor->getGearing() / 100);
	else
		motor->moveVoltage(speed * 120);

	if (motor == leftMotors)
		leftPrev = speed;
	else
		rightPrev = speed;
}

void setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors->setBrakeMode(b);
	rightMotors->setBrakeMode(b);
	motorMove(leftMotors, 0, true);
	motorMove(rightMotors, 0, true);
}

/**************************************************/
// speed control
double limitSpeed(double speed, double max) {
	if (speed > max)
		speed = max;
	if (speed < -max)
		speed = -max;

	return speed;
}

double slew(double target_speed, double step, double current_speed) {

	if (fabs(current_speed) > fabs(target_speed))
		step = 200;

	if (target_speed > current_speed + step)
		current_speed += step;
	else if (target_speed < current_speed - step)
		current_speed -= step;
	else
		current_speed = target_speed;

	return current_speed;
}

/**************************************************/
// settling
void waitUntilFinished(double exit_error) {
	switch (pid::mode) {
	case TRANSLATIONAL:
		while (odom::getDistanceError(
		           purepursuit::waypoints[purepursuit::waypoints.size() - 1]) >
		       exit_error) {
			pros::delay(10);
		}
		break;
	case ANGULAR:
		while (fabs(odom::getHeading() - pid::angularTarget) > exit_error)
			pros::delay(10);
		break;
	}
}

/**************************************************/
// translational movement
void move(std::vector<Point> waypoints, double max, double exit_error,
          double lp, double ap, MoveFlags flags) {
	pid::mode = TRANSLATIONAL;
	purepursuit::waypoints = std::vector{virtualPosition};

	for (int i = 0; i < waypoints.size(); i++) {
		if (flags & RELATIVE) {
			Point p = odom::getPosition();     // robot position
			double h = odom::getHeading(true); // robot heading in radians
			waypoints[i].x += p.x * cos(h) + p.y * sin(h);
			waypoints[i].y += p.y * cos(h) + p.x * sin(h);
		}

		purepursuit::waypoints.push_back(waypoints[i]);
	}

	purepursuit::reset(); // set the intialconditions

	virtualPosition = waypoints[waypoints.size() - 1];
	maxSpeed = max;
	pid::linearKP = lp;
	pid::angularKP = ap;
	pid::thru = (flags & THRU);
	pid::reverse = (flags & REVERSE);
	
	// reset the integrals
	pid::in_lin = 0;
	pid::in_ang = 0;

	if (!(flags & ASYNC)) {
		waitUntilFinished(exit_error);
		pid::mode = DISABLE;
		if (!(flags & THRU))
			chassis::setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	} else {
		pros::Task moveAsyncTask([exit_error]() {
			waitUntilFinished(exit_error);
			pid::mode = DISABLE;
		});
	}
}

void move(std::vector<Point> waypoints, double max, double exit_error,
          MoveFlags flags) {
	move(waypoints, max, exit_error, -1, -1, flags);
}

void move(std::vector<Point> waypoints, double max, MoveFlags flags) {
	move(waypoints, max, linear_exit_error, -1, -1, flags);
}

void move(std::vector<Point> waypoints, MoveFlags flags) {
	move(waypoints, 100.0, linear_exit_error, -1, -1, flags);
}

/**************************************************/
// rotational movement
void turn(double target, double max, double exit_error, double ap,
          MoveFlags flags) {
	pid::mode = ANGULAR;

	double bounded_heading = (int)(odom::getHeading()) % 360;

	double diff = target - bounded_heading;

	if (diff > 180)
		diff -= 360;
	else if (diff < -180)
		diff += 360;

	if (flags & RELATIVE) {
		diff = target;
	}

	double true_target = diff + odom::getHeading();

	pid::angularTarget = true_target;
	maxSpeed = max;
	pid::angularKP = ap;
	pid::in_ang = 0; // reset the integral value to zero

	if (!(flags & ASYNC)) {
		waitUntilFinished(exit_error);
		pid::mode = DISABLE;
		if (!(flags & THRU))
			chassis::setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	}
}

void turn(double target, double max, double exit_error, MoveFlags flags) {
	turn(target, max, exit_error, -1, flags);
}

void turn(double target, double max, MoveFlags flags) {
	turn(target, max, angular_exit_error, -1, flags);
}

void turn(double target, MoveFlags flags) {
	turn(target, 100, angular_exit_error, -1, flags);
}

void turn(Point target, double max, double exit_error, double ap,
          MoveFlags flags) {
	double angle_error = odom::getAngleError(target);
	turn(angle_error, max, exit_error, ap, flags);
}

void turn(Point target, double max, double exit_error, MoveFlags flags) {
	turn(target, max, exit_error, -1, flags);
}

void turn(Point target, double max, MoveFlags flags) {
	turn(target, max, angular_exit_error, -1, flags);
}

void turn(Point target, MoveFlags flags) {
	turn(target, 100, angular_exit_error, -1, flags);
}

/**************************************************/
// task control
int chassisTask() {
	while (1) {
		pros::delay(10);

		std::array<double, 2> speeds = {0, 0}; // left, right

		if (pid::mode == TRANSLATIONAL)
			speeds = pid::translational();
		else if (pid::mode == ANGULAR)
			speeds = pid::angular();
		else
			speeds = {leftDriveSpeed, rightDriveSpeed};

		// speed limiting
		speeds[0] = limitSpeed(speeds[0], maxSpeed);
		speeds[1] = limitSpeed(speeds[1], maxSpeed);

		// slew
		speeds[0] = slew(speeds[0], slew_step, leftPrev);
		speeds[1] = slew(speeds[1], slew_step, rightPrev);

		// output
		motorMove(leftMotors, speeds[0], false);
		motorMove(rightMotors, speeds[1], false);
	}
}

/**************************************************/
// initialization
void init(std::initializer_list<okapi::Motor> leftMotors,
          std::initializer_list<okapi::Motor> rightMotors, int gearset,
          double distance_constant, double degree_constant, double slew_step,
          double linear_exit_error, double angular_exit_error) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::slew_step = slew_step;
	chassis::linear_exit_error = linear_exit_error;
	chassis::angular_exit_error = angular_exit_error;

	// configure chassis motors
	chassis::leftMotors = std::make_shared<okapi::MotorGroup>(leftMotors);
	chassis::rightMotors = std::make_shared<okapi::MotorGroup>(rightMotors);
	chassis::leftMotors->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::rightMotors->setGearing((okapi::AbstractMotor::gearset)gearset);

	pros::Task chassis_task(chassisTask);
}

/**************************************************/
// operator control
void tank(double left_speed, double right_speed, bool velocity) {
	pid::mode = DISABLE; // turns off autonomous tasks
	chassis::leftDriveSpeed = left_speed;
	chassis::rightDriveSpeed = right_speed;
}

void arcade(double vertical, double horizontal, bool velocity) {
	pid::mode = DISABLE; // turns off autonomous task
	chassis::leftDriveSpeed = vertical + horizontal;
	chassis::rightDriveSpeed = vertical - horizontal;
}

} // namespace arms::chassis

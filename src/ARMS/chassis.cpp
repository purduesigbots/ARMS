#include "ARMS/api.h"
#include "api.h"

using namespace pros;

namespace arms::chassis {

// imu
std::shared_ptr<Imu> imu;

// chassis motors
std::shared_ptr<okapi::MotorGroup> leftMotors;
std::shared_ptr<okapi::MotorGroup> rightMotors;

// quad encoders
std::shared_ptr<ADIEncoder> leftEncoder;
std::shared_ptr<ADIEncoder> rightEncoder;
std::shared_ptr<ADIEncoder> middleEncoder;

// distance constants
double distance_constant; // ticks per inch
double degree_constant;   // ticks per degree

// slew control (autonomous only)
double slew_step; // smaller number = more slew

// default exit error
double default_exit_error;

// chassis variables
double maxSpeed = 100;
double leftPrev = 0;
double rightPrev = 0;

/**************************************************/
// basic control

// move motor group
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

void resetAngle(double angle) {
	if (imu)
		imu->set_rotation(angle);
}

void reset() {
	// reset odom
	odom::prev_left_pos = 0;
	odom::prev_right_pos = 0;
	odom::prev_middle_pos = 0;

	motorMove(leftMotors, 0, true);
	motorMove(rightMotors, 0, true);
	delay(10);

	leftMotors->tarePosition();
	rightMotors->tarePosition();

	if (leftEncoder) {
		leftEncoder->reset();
		rightEncoder->reset();
	}
	if (middleEncoder) {
		middleEncoder->reset();
	}
}

std::array<double, 2> getEncoders() {
	std::array<double, 2> encoders;

	if (leftEncoder) {
		encoders[0] = leftEncoder->get_value();
		encoders[1] = rightEncoder->get_value();
	} else {
		encoders[0] = leftMotors->getPosition();
		encoders[1] = rightMotors->getPosition();
	}

	return encoders;
}

double distance() {
	return (getEncoders()[0] + getEncoders()[1]) / 2 / distance_constant;
}

double angle() {
	if (imu) {
		return imu->get_rotation();
	} else {
		return (getEncoders()[0] - getEncoders()[1]) / 2 / degree_constant;
	}
}

/**************************************************/
// speed control
double limitSpeed(double speed, double max) {
	// speed limiting
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
// autonomous functions

// conditional waiting
void waitUntilFinished(double exit_error) {
	if (exit_error == 0)
		exit_error = default_exit_error;

	switch (pid::mode) {
	case LINEAR:
		while (fabs(distance() - pid::linearTarget) > exit_error)
			delay(10);
		break;
	case ANGULAR:
		while (fabs(angle() - pid::angularTarget) > exit_error)
			delay(10);
		break;
	case ODOM:
		while (odom::getDistanceError(pid::pointTarget) > exit_error)
			delay(10);
		break;
	}
}

// linear movement
void move(double target, double max, double exitError, double kp,
          flags_t flags) {
	reset();
	pid::mode = LINEAR;
	pid::linearTarget = target;
	maxSpeed = max;
	pid::linearKP = kp;
	pid::thru = (flags & THRU);

	if (!(flags & ASYNC))
		waitUntilFinished(exitError);
}

void move(double target, double max, double exitError, flags_t flags) {
	move(target, max, exitError, PID_DEFAULT, flags);
}

void move(double target, double max, flags_t flags) {
	move(target, max, EXIT_ERROR, PID_DEFAULT, flags);
}

void move(double target, flags_t flags) {
	move(target, 100.0, EXIT_ERROR, PID_DEFAULT, flags);
}

// odometry movement
void move(Point target, double max, double exit_error, double lp, double ap,
          flags_t flags) {
	reset();
	pid::mode = ODOM;
	pid::pointTarget = target.std(); // TODO: Update PID to use Point
	maxSpeed = max;
	pid::linearKP = lp;
	pid::angularKP = ap;
	pid::thru = (flags & THRU);

	if (!(flags & ASYNC))
		waitUntilFinished(exit_error);
}

void move(Point target, double max, double exit_error, flags_t flags) {
	move(target, max, exit_error, PID_DEFAULT, PID_DEFAULT, flags);
}

void move(Point target, double max, flags_t flags) {
	move(target, max, EXIT_ERROR, PID_DEFAULT, PID_DEFAULT, flags);
}

void move(Point target, flags_t flags) {
	move(target, 100.0, EXIT_ERROR, PID_DEFAULT, PID_DEFAULT, flags);
}

// rotational movement
void turn(double target, int max, double exit_error, double ap, flags_t flags) {
	reset();
	pid::mode = ANGULAR;

	if (flags & ABSOLUTE) {
		// convert from absolute to relative set point
		target -= (int)angle() % 360;

		// make sure all turns take most efficient route
		if (target > 180)
			target -= 360;
		else if (target < -180)
			target += 360;
	}

	pid::angularTarget = target;
	maxSpeed = max;
	pid::angularKP = ap;

	if (!(flags & ASYNC))
		waitUntilFinished(exit_error);
}

void turn(double target, int max, double exit_error, flags_t flags) {
	turn(target, max, exit_error, PID_DEFAULT, flags);
}

void turn(double target, int max, flags_t flags) {
	turn(target, max, EXIT_ERROR PID_DEFAULT, flags);
}

void turn(double target, flags_t flags) {
	turn(target, 100, EXIT_ERROR, PID_DEFAULT, flags);
}

// odometry turn to to point
void turn(Point target, int max, double exit_error, double ap, flags_t flags) {
	double angle_error = odom::getAngleError(target.std());
	turn(angle_error, max, exit_error, ap, flags | ABSOLUTE);
}

void turn(Point target, int max, double exit_error, flags_t flags) {
	turn(target, max, exit_error, PID_DEFAULT, flags);
}

void turn(Point target, int max, flags_t flags) {
	turn(target, max, EXIT_ERROR, PID_DEFAULT, flags);
}

void turn(Point target, flags_t flags) {
	turn(target, 100, EXIT_ERROR, PID_DEFAULT, flags);
}

/**************************************************/
// task control
int chassisTask() {
	while (1) {
		delay(10);

		std::array<double, 2> speeds = {0, 0}; // left, right

		if (pid::mode == LINEAR) {
			speeds = pid::linear();
		} else if (pid::mode == ANGULAR) {
			speeds = pid::angular();
		} else if (pid::mode == ODOM) {
			speeds = pid::odom();
		} else {
			continue;
		}

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
std::shared_ptr<ADIEncoder> initEncoder(int encoderPort, int expanderPort) {
	std::shared_ptr<ADIEncoder> encoder;

	bool reversed = encoderPort > 0 ? false : true;

	int encoderPort2 =
	    abs((encoderPort > 0) ? (abs(encoderPort) + 1) : encoderPort--);
	encoderPort = abs(encoderPort);

	if (expanderPort != 0) {
		std::tuple<int, int, int> pair(expanderPort, encoderPort, encoderPort2);
		encoder = std::make_shared<ADIEncoder>(pair, reversed);
	} else {
		encoder = std::make_shared<ADIEncoder>(encoderPort, encoderPort2, reversed);
	}

	return encoder;
}

void init(std::initializer_list<okapi::Motor> leftMotors,
          std::initializer_list<okapi::Motor> rightMotors, int gearset,
          double distance_constant, double degree_constant, double slew_tep,
          double arc_slew_step, int imuPort,
          std::tuple<int, int, int> encoderPorts, int expanderPort,
          double exit_error) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::slew_step = slew_step;
	chassis::arc_slew_step = arc_slew_step;
	chassis::default_exit_error = exit_error;

	// configure chassis motors
	chassis::leftMotors = std::make_shared<okapi::MotorGroup>(leftMotors);
	chassis::rightMotors = std::make_shared<okapi::MotorGroup>(rightMotors);
	chassis::leftMotors->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::rightMotors->setGearing((okapi::AbstractMotor::gearset)gearset);

	// initialize imu
	if (imuPort != 0) {
		imu = std::make_shared<Imu>(imuPort);
		delay(2000); // wait for IMU intialization
		imu->reset();
	}

	chassis::leftMotors->tarePosition();
	chassis::rightMotors->tarePosition();

	if (std::get<0>(encoderPorts) != 0) {
		leftEncoder = initEncoder(std::get<0>(encoderPorts), expanderPort);
	}

	if (std::get<1>(encoderPorts) != 0) {
		rightEncoder = initEncoder(std::get<1>(encoderPorts), expanderPort);
	}

	if (std::get<2>(encoderPorts) != 0) {
		middleEncoder = initEncoder(std::get<2>(encoderPorts), expanderPort);
	}

	Task chassis_task(chassisTask);
}

/**************************************************/
// operator control
void tank(double left_speed, double right_speed, bool velocity) {
	pid::mode = DISABLE; // turns off autonomous tasks

	motorMove(leftMotors, left_speed, velocity);
	motorMove(rightMotors, right_speed, velocity);
}

void arcade(double vertical, double horizontal, bool velocity) {
	pid::mode = DISABLE; // turns off autonomous task

	motorMove(leftMotors, vertical + horizontal, velocity);
	motorMove(rightMotors, vertical - horizontal, velocity);
}

} // namespace arms::chassis

#include "api.h"
#include "arms/api.h"

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
double distance_constant; // ticks per foot
double degree_constant;   // ticks per degree

// slew control (autonomous only)
double slew_step; // smaller number = more slew

// chassis variables
double maxSpeed = 100;
double exit_error = 0;
double leftPrev = 0;
double rightPrev = 0;
bool useVelocity = false;

/**************************************************/
// basic control

// move motor group
void motorMove(std::shared_ptr<okapi::MotorGroup> motor, double speed,
               bool vel = useVelocity) {
	if (vel)
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

double slew(double target_speed, double step, double* current_speed) {

	if (fabs(*current_speed) > fabs(target_speed))
		step = 200;

	if (target_speed > *current_speed + step)
		*current_speed += step;
	else if (target_speed < *current_speed - step)
		*current_speed -= step;
	else
		*current_speed = target_speed;

	return *current_speed;
}

/**************************************************/
// conditional waiting
void waitUntilAtDistance() {
	while (fabs(position() - pid::linearTarget) > exit_error)
		delay(10);
}

void waitUntilAtAngle() {
	while (fabs(angle() - pid::angularTarget) > exit_error)
		delay(10);
}

void waitUntilAtTarget() {
	while (odom::getDistanceError(pid::pointTarget) > exit_error)
		delay(10);
}

/**************************************************/
// autonomous functions
void moveAsync(double sp, int max) {
	pid::mode = LINEAR;
	reset();
	maxSpeed = max;
	pid::linearTarget = sp;
}

void moveAsync(std::array<double, 2> sp, double max) {
	pid::mode = ODOM;
	reset();
	maxSpeed = max;
	pid::pointTarget = point;
}

void turnAsync(double sp, int max) {
	pid::mode = ANGULAR;
	reset();
	sp += angle();
	maxSpeed = max;
	pid::angularTarget = sp;
}

void turnAbsoluteAsync(double sp, int max) {
	pid::mode = ANGULAR;

	// convert from absolute to relative set point
	sp = sp - (int)angle() % 360;

	// make sure all turns take most efficient route
	if (sp > 180)
		sp -= 360;
	else if (sp < -180)
		sp += 360;

	turnAsync(sp, max);
}

void move(double sp, double max, double linear_kp, double angle_kp,
          double exit_error) {
	moveAsync(sp, max);
	waitUntilAtDistance();
}

void move(std::array<double, 2> sp, double max, double linear_kp,
          double angle_kp, double exit_error, bool reverse) {
	moveAsync(sp, max);
	waitUntilAtTarget();
}

void turn(double sp, int max) {
	turnAsync(sp, max);
	waitUntilAtAngle();
}

void turnAbsolute(double sp, int max) {
	turnAbsoluteAsync(sp, max);
	waitUntilAtAngle();
}

void moveThru(std::array<double, 1> sp, double max) {
	moveAsync(sp, max);
	waitUntilAtDistance();
}

void moveThru(std::array<double, 2> sp, double max, double exit_error,
              int reverse) {
	moveAsync(sp, max, exit_error, reverse);
	waitUntilAtTarget();
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
		} else if (pid::mode == ODOM || pid::mode == ODOM_THRU) {
			speeds = pid::odom();
		} else {
			continue;
		}

		double leftSpeed = speeds[0];
		double rightSpeed = speeds[1];

		// speed limiting
		leftSpeed = limitSpeed(leftSpeed, maxSpeed);
		rightSpeed = limitSpeed(rightSpeed, maxSpeed);

		// slew
		leftSpeed = slew(leftSpeed, slew_step, &leftPrev);
		rightSpeed = slew(rightSpeed, slew_step, &rightPrev);

		// output
		motorMove(leftMotors, leftSpeed);
		motorMove(rightMotors, rightSpeed);
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
          double distance_constant, double degree_constant, int settle_time,
          double settle_threshold_linear, double settle_threshold_angular,
          double slew_tep, double arc_slew_step, int imuPort,
          std::tuple<int, int, int> encoderPorts, int expanderPort) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::slew_step = slew_step;
	chassis::arc_slew_step = arc_slew_step;

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
void tank(double left_speed, double right_speed) {
	pid::mode = DISABLE; // turns off autonomous tasks

	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed, false);
}

void arcade(double vertical, double horizontal) {
	pid::mode = DISABLE; // turns off autonomous task

	motorMove(leftMotors, vertical + horizontal, false);
	motorMove(rightMotors, vertical - horizontal, false);
}

void holonomic(double y, double x, double z) {
	pid::mode = DISABLE; // turns off autonomous task

	motorMove(frontLeft, y + x + z, false);
	motorMove(frontRight, y - x - z, false);
	motorMove(backLeft, y - x + z, false);
	motorMove(backRight, y + x - z, false);
}

} // namespace arms::chassis

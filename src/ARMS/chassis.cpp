#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "ARMS/pid.h"
#include "api.h"

using namespace pros;

namespace chassis {

// imu
std::shared_ptr<Imu> imu;

// chassis motors
std::shared_ptr<okapi::MotorGroup> leftMotors;
std::shared_ptr<okapi::MotorGroup> rightMotors;

// individual motors
std::shared_ptr<okapi::Motor> frontLeft;
std::shared_ptr<okapi::Motor> frontRight;
std::shared_ptr<okapi::Motor> backLeft;
std::shared_ptr<okapi::Motor> backRight;

// quad encoders
std::shared_ptr<ADIEncoder> leftEncoder;
std::shared_ptr<ADIEncoder> rightEncoder;
std::shared_ptr<ADIEncoder> middleEncoder;

// distance constants
double distance_constant; // ticks per foot
double degree_constant;   // ticks per degree

// settle constants
int settle_count;
int settle_time;
double settle_threshold_linear;
double settle_threshold_angular;

// slew control (autonomous only)
double accel_step; // smaller number = more slew
double arc_step;   // acceleration for arcs

// chassis variables
double maxSpeed = 100;
double leftPrev = 0;
double rightPrev = 0;
bool useVelocity = false;

/**************************************************/
// basic control

// move motor group at given velocity
void motorMove(std::shared_ptr<okapi::MotorGroup> motor, int speed,
               bool vel = useVelocity) {
	if (vel)
		motor->moveVelocity(vel * (double)motor->getGearing() / 200);
	else
		motor->moveVoltage(speed * 120);
}

void motorMove(std::shared_ptr<okapi::Motor> motor, int speed,
               bool vel = useVelocity) {
	if (vel)
		motor->moveVelocity(vel * (double)motor->getGearing() / 200);
	else
		motor->moveVoltage(speed * 120);
}

void setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors->setBrakeMode(b);
	rightMotors->setBrakeMode(b);
	motorMove(leftMotors, 0, true);
	motorMove(rightMotors, 0, true);
}

void reset() {

	// reset odom
	odom::prev_left_pos = 0;
	odom::prev_right_pos = 0;
	odom::prev_middle_pos = 0;

	leftPrev = 0;
	rightPrev = 0;

	settle_count = 0;

	pid::vectorAngle = 0;

	motorMove(leftMotors, 0, true);
	motorMove(rightMotors, 0, true);
	delay(10);
	leftMotors->tarePosition();
	rightMotors->tarePosition();
	frontLeft->tarePosition();
	frontRight->tarePosition();
	backLeft->tarePosition();
	backRight->tarePosition();
	if (leftEncoder) {
		leftEncoder->reset();
		rightEncoder->reset();
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

double position(bool yDirection) {
	if (yDirection) {
		double top_pos, bot_pos;

		if (middleEncoder) {
			top_pos = middleEncoder->get_value();
			bot_pos = middleEncoder->get_value();
		} else {
			top_pos = frontLeft->getPosition() - frontRight->getPosition();
			bot_pos = backRight->getPosition() - backLeft->getPosition();
		}

		return (top_pos + bot_pos) / 2;

	} else {
		return (getEncoders()[0] + getEncoders()[1]) / 2;
	}
}

double angle() {
	if (imu) {
		return -imu->get_rotation();
	} else {
		return (-getEncoders()[0] + getEncoders()[1]) / 2;
	}
}

double difference() {
	return (getEncoders()[0] - getEncoders()[1]);
}

/**************************************************/
// speed control
double limitSpeed(double speed) {
	// speed limiting
	if (speed > maxSpeed)
		speed = maxSpeed;
	if (speed < -maxSpeed)
		speed = -maxSpeed;

	return speed;
}

double slew(double target_speed, double step, double* current_speed) {

	if (abs(*current_speed) > abs(target_speed))
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
// chassis settling
int wheelMoving(double sv, double* psv) {
	int stop = 0;
	double thresh = settle_threshold_linear;
	if (pid::mode == ANGULAR)
		thresh = settle_threshold_angular;

	if (abs(sv - *psv) < thresh)
		stop = 1;

	*psv = sv;

	return stop;
}

bool settled() {
	static double psv_left = 0;
	static double psv_right = 0;
	static double psv_middle = 0;

	int wheelMovingCount = 0;

	if (leftEncoder) {
		wheelMovingCount += wheelMoving(leftEncoder->get_value(), &psv_left);
		wheelMovingCount += wheelMoving(rightEncoder->get_value(), &psv_right);
	} else {
		wheelMovingCount += wheelMoving(leftMotors->getPosition(), &psv_left);
		wheelMovingCount += wheelMoving(rightMotors->getPosition(), &psv_right);
	}

	wheelMovingCount += wheelMoving(position(true), &psv_middle);

	if (wheelMovingCount == 0)
		settle_count++;
	else
		settle_count = 0;

	// not driving if we haven't moved
	if (settle_count > settle_time)
		return true;
	else
		return false;
}

void waitUntilSettled() {
	while (!settled())
		delay(10);
}

/**************************************************/
// autonomous functions
void moveAsync(double sp, int max) {
	sp *= distance_constant;
	reset();
	maxSpeed = max;
	pid::linearTarget = sp;
	pid::mode = LINEAR;
}

void turnAsync(double sp, int max) {
	pid::mode = ANGULAR;

	if (imu)
		sp += position();
	else
		sp *= degree_constant;

	reset();
	maxSpeed = max;
	pid::angularTarget = sp;
}

void turnAbsoluteAsync(double sp, int max) {
	pid::mode = ANGULAR;

	// convert from absolute to relative set point
	sp = sp - (int)position() % 360;

	// make sure all turns take most efficient route
	if (sp > 180)
		sp -= 360;
	else if (sp < -180)
		sp += 360;

	turnAsync(sp, max);
}

void holoAsync(double distance, double angle, int max) {
	distance *= distance_constant;
	reset();
	maxSpeed = max;
	pid::linearTarget = distance;
	pid::vectorAngle = angle * M_PI / 180;
	pid::mode = LINEAR;
}

void move(double sp, int max) {
	moveAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void turn(double sp, int max) {
	turnAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void turnAbsolute(double sp, int max) {
	turnAbsoluteAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void holo(double distance, double angle, int max) {
	holoAsync(distance, angle, max);
	delay(450);
	waitUntilSettled();
}

void fast(double sp, int max) {
	double speed; // speed before PID

	if (sp < 0)
		max = -max;
	reset();
	pid::mode = DISABLE;

	while (abs(position()) < abs(sp * distance_constant)) {
		speed = slew(max, accel_step, &leftPrev);
		// differential PID
		double dif = difference() * pid::difKP;
		motorMove(leftMotors, speed - dif);
		motorMove(rightMotors, speed + dif);
		delay(20);
	}
}

void voltage(int t, int left_speed, int right_speed) {
	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed == 0 ? left_speed : right_speed, false);
	delay(t);
}

void velocity(int t, int max) {
	motorMove(leftMotors, max, true);
	motorMove(rightMotors, max, true);
	delay(t);
}

void arc(bool mirror, int arc_length, double rad, int max, int type) {
	reset();
	int time_step = 0;
	pid::mode = DISABLE;
	bool reversed = false;

	// reverse the movement if the length is negative
	if (arc_length < 0) {
		reversed = true;
		arc_length = -arc_length;
	}

	// fix jerk bug between velocity movements
	if (type < 2) {
		motorMove(leftMotors, 0, true);
		motorMove(rightMotors, 0, true);
		delay(10);
	}

	while (time_step < arc_length) {

		// speed
		int error = arc_length - time_step;
		double speed = error * pid::arcKP;

		if (type == 1 || type == 2)
			speed = max;

		// speed limiting
		if (speed > max)
			speed = max;
		if (speed < -max)
			speed = -max;

		// prevent backtracking
		if (speed < 0)
			speed = 0;

		speed = slew(speed, accel_step, &leftPrev); // slew

		if (reversed)
			speed = -speed;

		double scaled_speed = speed * rad;

		if (type == 1)
			scaled_speed *= (double)time_step / arc_length;
		else if (type == 2)
			scaled_speed *= std::abs(2 * (.5 - (double)time_step / arc_length));
		else if (type == 3)
			scaled_speed *= (1 - (double)time_step / arc_length);

		// assign chassis motor speeds
		motorMove(leftMotors, mirror ? speed : scaled_speed, true);
		motorMove(rightMotors, mirror ? scaled_speed : speed, true);

		// increment time step
		time_step += 10;
		delay(10);
	}

	if (type != 1 && type != 2) {
		motorMove(leftMotors, 0, true);
		motorMove(rightMotors, 0, true);
	}
}

void arcLeft(int arc_length, double rad, int max, int type) {
	arc(false, arc_length, rad, max, type);
}

void arcRight(int arc_length, double rad, int max, int type) {
	arc(true, arc_length, rad, max, type);
}

void scurve(bool mirror, int arc1, int mid, int arc2, int max) {

	// first arc
	arc(mirror, arc1, 1, max, 1);

	// middle movement
	velocity(mid, max);

	// final arc
	arc(!mirror, arc2, 1, max, 2);
}

void sLeft(int arc1, int mid, int arc2, int max) {
	scurve(false, arc1, mid, arc2, max);
}

void sRight(int arc1, int mid, int arc2, int max) {
	scurve(true, arc1, mid, arc2, max);
}

void _sLeft(int arc1, int mid, int arc2, int max) {
	scurve(true, -arc1, mid, -arc2, -max);
}

void _sRight(int arc1, int mid, int arc2, int max) {
	scurve(false, -arc1, -mid, -arc2, max);
}

/**************************************************/
// task control
int chassisTask() {

	while (1) {
		delay(20);

		double leftSpeed = 0;
		double rightSpeed = 0;

		if (pid::mode == LINEAR) {
			leftSpeed = pid::linear();
			rightSpeed = pid::linear(true); // dif pid for right side
		} else if (pid::mode == ANGULAR) {
			rightSpeed = pid::angular();
			leftSpeed = -rightSpeed;
		} else if (pid::mode == ODOM) {
			std::array<double, 2> speeds = pid::odom();
			leftSpeed = speeds[0];
			rightSpeed = speeds[1];
		} else {
			continue;
		}

		// speed limiting
		leftSpeed = limitSpeed(leftSpeed);
		rightSpeed = limitSpeed(rightSpeed);

		// slew
		leftSpeed = slew(leftSpeed, accel_step, &leftPrev);
		rightSpeed = slew(rightSpeed, accel_step, &rightPrev);

		// set motors
		if (pid::vectorAngle != 0) {
			// calculate vectors for each wheel set
			double frontVector = sin(M_PI / 4 - pid::vectorAngle);
			double backVector = sin(M_PI / 4 + pid::vectorAngle);

			// set scaling factor based on largest vector
			double largestVector;
			if (abs(frontVector) > abs(backVector)) {
				largestVector = abs(frontVector);
			} else {
				largestVector = abs(backVector);
			}

			double largestSpeed;
			if (leftSpeed > rightSpeed)
				largestSpeed = leftSpeed;
			else
				largestSpeed = rightSpeed;

			double scalingFactor = largestSpeed / largestVector;

			frontVector *= scalingFactor;
			backVector *= scalingFactor;

			motorMove(frontLeft, frontVector);
			motorMove(backLeft, backVector);
			motorMove(frontRight, backVector);
			motorMove(backRight, frontVector);

		} else {
			motorMove(leftMotors, leftSpeed);
			motorMove(rightMotors, rightSpeed);
		}
	}
}

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
          double accel_step, double arc_step, int imuPort,
          std::tuple<int, int, int> encoderPorts, int expanderPort) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::settle_count = settle_count;
	chassis::settle_threshold_linear = settle_threshold_linear;
	chassis::settle_threshold_angular = settle_threshold_angular;
	chassis::accel_step = accel_step;
	chassis::arc_step = arc_step;

	// configure chassis motors
	chassis::leftMotors = std::make_shared<okapi::MotorGroup>(leftMotors);
	chassis::rightMotors = std::make_shared<okapi::MotorGroup>(rightMotors);
	chassis::leftMotors->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::rightMotors->setGearing((okapi::AbstractMotor::gearset)gearset);

	// initialize imu
	if (imuPort != 0) {
		imu = std::make_shared<Imu>(imuPort);
		imu->reset();
		delay(1500);
		while (imu->is_calibrating()) {
			delay(10);
		}
		delay(1000);
		// printf("IMU calibrated!");
	}
	// configure individual motors for holonomic chassis
	chassis::frontLeft = std::make_shared<okapi::Motor>(*leftMotors.begin());
	chassis::backLeft = std::make_shared<okapi::Motor>(*(leftMotors.end() - 1));
	chassis::frontRight = std::make_shared<okapi::Motor>(*rightMotors.begin());
	chassis::backRight = std::make_shared<okapi::Motor>(*(rightMotors.end() - 1));

	// set gearing for individual motors
	chassis::frontLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::backLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::frontRight->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::backRight->setGearing((okapi::AbstractMotor::gearset)gearset);

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
void tank(int left_speed, int right_speed) {
	pid::mode = DISABLE; // turns off autonomous tasks
	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed, false);
}

void arcade(int vertical, int horizontal) {
	pid::mode = DISABLE; // turns off autonomous task
	motorMove(leftMotors, vertical + horizontal, false);
	motorMove(rightMotors, vertical - horizontal, false);
}

void holonomic(int x, int y, int z) {
	pid::mode = 0; // turns off autonomous task
	motorMove(frontLeft, x + y + z, false);
	motorMove(frontRight, x - y - z, false);
	motorMove(backLeft, x + y - z, false);
	motorMove(backRight, x - y + z, false);
}

} // namespace chassis

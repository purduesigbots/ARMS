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
double maxAngular = 50; // holonomic odom only
double leftPrev = 0;
double rightPrev = 0;
bool useVelocity = false;

// joystick threshold
int joystick_threshold;

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
double limitSpeed(double speed, double max) {
	// speed limiting
	if (speed > max)
		speed = max;
	if (speed < -max)
		speed = -max;

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
	int isMoving = 0;
	double thresh = settle_threshold_linear;
	if (pid::mode == ANGULAR)
		thresh = settle_threshold_angular;

	if (abs(sv - *psv) > thresh)
		isMoving = 1;

	*psv = sv;

	return isMoving;
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
		delay(10);
	}
}

void voltage(int t, int left_speed, int right_speed) {
	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed == 101 ? left_speed : right_speed, false);
	delay(t);
}

void velocity(int t, int left_max, int right_max) {
	motorMove(leftMotors, left_max, true);
	motorMove(rightMotors, right_max == 101 ? left_max : right_max, true);
	delay(t);
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
			speeds[1] = pid::angular();
			speeds[0] = -speeds[1];
		} else if (pid::mode == ODOM || pid::mode == ODOM_HOLO) {
			speeds = pid::odom();
		} else {
			continue;
		}

		double leftSpeed = speeds[0];
		double rightSpeed = speeds[1];

		// speed limiting
		leftSpeed = limitSpeed(leftSpeed, maxSpeed);
		rightSpeed = limitSpeed(rightSpeed, maxSpeed);

		// set motors
		if (pid::vectorAngle != 0) {
			// calculate vectors for each wheel set
			double frontVector = sin(M_PI / 4 - pid::vectorAngle);
			double backVector = sin(M_PI / 4 + pid::vectorAngle);

			// set scaling factor based on largest vector
			double largestVector;
			if (fabs(frontVector) > fabs(backVector)) {
				largestVector = frontVector;
			} else {
				largestVector = backVector;
			}

			double largestSpeed;
			if (fabs(leftSpeed) > fabs(rightSpeed))
				largestSpeed = leftSpeed;
			else
				largestSpeed = rightSpeed;

			double scalingFactor = fabs(largestSpeed) / fabs(largestVector);

			frontVector *= scalingFactor;
			backVector *= scalingFactor;

			double turnSpeed = rightSpeed - leftSpeed;
			turnSpeed = limitSpeed(turnSpeed, 50);

			motorMove(frontLeft, frontVector - turnSpeed);
			motorMove(backLeft, backVector - turnSpeed);
			motorMove(frontRight, backVector + turnSpeed);
			motorMove(backRight, frontVector + turnSpeed);

		} else {
			leftSpeed = slew(leftSpeed, accel_step, &leftPrev);
			rightSpeed = slew(rightSpeed, accel_step, &rightPrev);

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
          std::tuple<int, int, int> encoderPorts, int expanderPort,
          int joystick_threshold) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::settle_time = settle_time;
	chassis::settle_threshold_linear = settle_threshold_linear;
	chassis::settle_threshold_angular = settle_threshold_angular;
	chassis::accel_step = accel_step;
	chassis::arc_step = arc_step;
	chassis::joystick_threshold = joystick_threshold;

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

	// apply thresholding
	left_speed = (abs(left_speed) > joystick_threshold ? left_speed : 0);
	right_speed = (abs(right_speed) > joystick_threshold ? right_speed : 0);

	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed, false);
}

void arcade(int vertical, int horizontal) {
	pid::mode = DISABLE; // turns off autonomous task

	// apply thresholding
	vertical = (abs(vertical) > joystick_threshold ? vertical : 0);
	horizontal = (abs(horizontal) > joystick_threshold ? horizontal : 0);

	motorMove(leftMotors, vertical + horizontal, false);
	motorMove(rightMotors, vertical - horizontal, false);
}

void holonomic(int x, int y, int z) {
	pid::mode = DISABLE; // turns off autonomous task

	// apply thresholding
	x = (abs(x) > joystick_threshold ? x : 0);
	y = (abs(y) > joystick_threshold ? y : 0);
	z = (abs(z) > joystick_threshold ? z : 0);

	motorMove(frontLeft, x + y + z, false);
	motorMove(frontRight, x - y - z, false);
	motorMove(backLeft, x + y - z, false);
	motorMove(backRight, x - y + z, false);
}

} // namespace chassis

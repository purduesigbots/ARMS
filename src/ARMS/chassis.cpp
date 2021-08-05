#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include "ARMS/odom.h"
#include "ARMS/pid.h"
#include "api.h"

using namespace pros;

namespace arms::chassis {

// move motor group at given velocity
void Chassis::motorMove(std::shared_ptr<okapi::MotorGroup> motor,
                        double speed) {
	if (useVelocity)
		motor->moveVelocity(speed * (double)motor->getGearing() / 100);
	else
		motor->moveVoltage(speed * 120);
}

void Chassis::motorMove(std::shared_ptr<okapi::Motor> motor, double speed) {
	if (useVelocity)
		motor->moveVelocity(speed * (double)motor->getGearing() / 100);
	else
		motor->moveVoltage(speed * 120);
}

// move motor group at given velocity
void Chassis::motorMove(std::shared_ptr<okapi::MotorGroup> motor, double speed,
                        bool vel) {
	if (vel)
		motor->moveVelocity(speed * (double)motor->getGearing() / 100);
	else
		motor->moveVoltage(speed * 120);
}

void Chassis::motorMove(std::shared_ptr<okapi::Motor> motor, double speed,
                        bool vel) {
	if (vel)
		motor->moveVelocity(speed * (double)motor->getGearing() / 100);
	else
		motor->moveVoltage(speed * 120);
}

void Chassis::setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors->setBrakeMode(b);
	rightMotors->setBrakeMode(b);
	motorMove(leftMotors, 0, true);
	motorMove(rightMotors, 0, true);
}

void Chassis::resetAngle(double angle) {
	if (imu)
		imu->set_rotation(angle);
}

void Chassis::reset() {

	settle_count = 0;

	pid.setVectorAngle(0);

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

std::array<double, 2> Chassis::getEncoders() {
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

double Chassis::position(bool yDirection) {
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

double Chassis::angle() {
	if (imu) {
		return imu->get_rotation();
	} else {
		return (getEncoders()[0] - getEncoders()[1]) / 2 / degree_constant;
	}
}

double Chassis::difference() {
	if (imu) {
		return angle() - pid.getAngularTarget();
	} else {
		return (getEncoders()[0] - getEncoders()[1]);
	}
}

/**************************************************/
// speed control
double Chassis::limitSpeed(double speed, double max) {
	// speed limiting
	if (speed > max)
		speed = max;
	if (speed < -max)
		speed = -max;

	return speed;
}

double Chassis::slew(double target_speed, double step, double* current_speed) {

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
// chassis settling
int Chassis::wheelMoving(double sv, double* psv) {
	int isMoving = 0;
	double thresh = settle_threshold_linear;
	if (pid.getMode() == ANGULAR)
		thresh = settle_threshold_angular;

	if (fabs(sv - *psv) > thresh)
		isMoving = 1;

	*psv = sv;

	return isMoving;
}

bool Chassis::settled() {
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

void Chassis::waitUntilSettled() {
	while (!settled())
		delay(10);
}

/**************************************************/
// autonomous functions
void Chassis::moveAsync(double sp, int max) {
	pid.setMode(LINEAR);
	sp *= distance_constant;
	reset();
	maxSpeed = max;
	pid.setAngularTarget(angle()); // hold the robot to the current angle
	pid.setLinearTarget(sp);
}

void Chassis::turnAsync(double sp, int max) {
	pid.setMode(ANGULAR);
	reset();
	sp += angle();
	maxSpeed = max;
	pid.setAngularTarget(sp);
}

void Chassis::turnAbsoluteAsync(double sp, int max) {
	pid.setMode(ANGULAR);

	// convert from absolute to relative set point
	sp = sp - (int)angle() % 360;

	// make sure all turns take most efficient route
	if (sp > 180)
		sp -= 360;
	else if (sp < -180)
		sp += 360;

	turnAsync(sp, max);
}

void Chassis::holoAsync(double distance, double angle, int max) {
	distance *= distance_constant;
	reset();
	maxSpeed = max;
	pid.setLinearTarget(distance);
	pid.setVectorAngle(angle * M_PI / 180);
	pid.setMode(LINEAR);
}

void Chassis::move(double sp, int max) {
	moveAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void Chassis::turn(double sp, int max) {
	turnAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void Chassis::turnAbsolute(double sp, int max) {
	turnAbsoluteAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void Chassis::holo(double distance, double angle, int max) {
	holoAsync(distance, angle, max);
	delay(450);
	waitUntilSettled();
}

void Chassis::fast(double sp, int max) {
	double speed; // speed before PID

	if (sp < 0)
		max = -max;
	reset();
	pid.setMode(DISABLE);

	while (fabs(position()) < fabs(sp * distance_constant)) {
		speed = slew(max, accel_step, &output_prev[0]);
		output_prev[1] = output_prev[2] = output_prev[3] = output_prev[0];
		// differential PID
		double dif = difference() * pid.getDifKP();
		motorMove(leftMotors, speed - dif);
		motorMove(rightMotors, speed + dif);
		delay(10);
	}
}

void Chassis::voltage(int t, int left_speed, int right_speed) {
	pid.setMode(DISABLE);
	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed == 101 ? left_speed : right_speed, false);
	delay(t);
}

void Chassis::velocity(int t, int left_max, int right_max) {
	pid.setMode(DISABLE);
	motorMove(leftMotors, left_max, true);
	motorMove(rightMotors, right_max == 101 ? left_max : right_max, true);
	delay(t);
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

void Chassis::startTask() {
	Task chassis_task([this](void) -> int {
		while (1) {
			delay(10);

			std::array<double, 2> speeds = {0, 0}; // left, right

			if (pid.getMode() == LINEAR) {
				speeds = pid.linear(position(), position(true), maxSpeed, difference());
			} else if (pid.getMode() == ANGULAR) {
				speeds = pid.angular(angle());
			} else if (pid.getMode() == ODOM || pid.getMode() == ODOM_HOLO ||
			           pid.getMode() == ODOM_HOLO_THRU) {
				speeds =
				    pid.odom(maxSpeed, global_x, global_y, heading, heading_degrees);
			} else {
				continue;
			}

			double leftSpeed = speeds[0];
			double rightSpeed = speeds[1];

			// speed limiting
			leftSpeed = limitSpeed(leftSpeed, maxSpeed);
			rightSpeed = limitSpeed(rightSpeed, maxSpeed);

			double output[4] = {0, 0, 0, 0};

			// set motors
			if (pid.getVectorAngle() != 0) {
				// calculate vectors for each wheel set
				double frontVector = sin(M_PI / 4 - pid.getVectorAngle());
				double backVector = sin(M_PI / 4 + pid.getVectorAngle());

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

				double scalingFactor;
				if (pid.getMode() == ODOM_HOLO_THRU)
					scalingFactor = fabs(maxSpeed) / fabs(largestVector);
				else
					scalingFactor = fabs(largestSpeed) / fabs(largestVector);

				frontVector *= scalingFactor;
				backVector *= scalingFactor;

				double turnSpeed = rightSpeed - leftSpeed;
				turnSpeed = limitSpeed(turnSpeed, maxTurn);

				output[0] = frontVector - turnSpeed; // front left
				output[1] = backVector - turnSpeed;  // back left
				output[2] = backVector + turnSpeed;  // front right
				output[3] = frontVector + turnSpeed; // back right

			} else {
				output[0] = output[1] = leftSpeed;  // left motors
				output[2] = output[3] = rightSpeed; // right motors
			}

			for (int i = 0; i < 4; i++) {
				output[i] = slew(output[i], accel_step, &output_prev[i]);
			}

			if (pid.getVectorAngle() != 0) {
				motorMove(frontLeft, output[0]);
				motorMove(backLeft, output[1]);
				motorMove(frontRight, output[2]);
				motorMove(backRight, output[3]);
			} else {
				motorMove(leftMotors, output[0]);
				motorMove(rightMotors, output[2]);
			}
		}
	});
}

Chassis::Chassis(std::initializer_list<okapi::Motor> leftMotorsList,
                 std::initializer_list<okapi::Motor> rightMotorsList,
                 int gearset, double distance_constant, double degree_constant,
                 int settle_time, double settle_threshold_linear,
                 double settle_threshold_angular, double accel_step,
                 double arc_step, int imuPort,
                 std::tuple<int, int, int> encoderPorts, int expanderPort,
                 int joystick_threshold, pid::PID pid) {

	// assign constants
	this->distance_constant = distance_constant;
	this->degree_constant = degree_constant;
	this->settle_time = settle_time;
	this->settle_threshold_linear = settle_threshold_linear;
	this->settle_threshold_angular = settle_threshold_angular;
	this->accel_step = accel_step;
	this->arc_step = arc_step;
	this->joystick_threshold = joystick_threshold;
	this->pid = pid;

	// configure chassis motors
	leftMotors = std::make_shared<okapi::MotorGroup>(leftMotorsList);
	rightMotors = std::make_shared<okapi::MotorGroup>(rightMotorsList);
	leftMotors->setGearing((okapi::AbstractMotor::gearset)gearset);
	rightMotors->setGearing((okapi::AbstractMotor::gearset)gearset);

	// initialize imu
	if (imuPort != 0) {
		imu = std::make_shared<Imu>(imuPort);
		imu->reset();
	}

	// configure individual motors for holonomic chassis
	frontLeft = std::make_shared<okapi::Motor>(*leftMotorsList.begin());
	backLeft = std::make_shared<okapi::Motor>(*(leftMotorsList.end() - 1));
	frontRight = std::make_shared<okapi::Motor>(*rightMotorsList.begin());
	backRight = std::make_shared<okapi::Motor>(*(rightMotorsList.end() - 1));

	// set gearing for individual motors
	frontLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	backLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	frontRight->setGearing((okapi::AbstractMotor::gearset)gearset);
	backRight->setGearing((okapi::AbstractMotor::gearset)gearset);

	if (std::get<0>(encoderPorts) != 0) {
		leftEncoder = initEncoder(std::get<0>(encoderPorts), expanderPort);
	}

	if (std::get<1>(encoderPorts) != 0) {
		rightEncoder = initEncoder(std::get<1>(encoderPorts), expanderPort);
	}

	if (std::get<2>(encoderPorts) != 0) {
		middleEncoder = initEncoder(std::get<2>(encoderPorts), expanderPort);
	}

	startTask();
}

/**************************************************/
// operator control
void Chassis::tank(double left_speed, double right_speed) {
	pid.setMode(DISABLE); // turns off autonomous tasks

	// apply thresholding
	left_speed = (fabs(left_speed) > joystick_threshold ? left_speed : 0);
	right_speed = (fabs(right_speed) > joystick_threshold ? right_speed : 0);

	motorMove(leftMotors, left_speed, false);
	motorMove(rightMotors, right_speed, false);
}

void Chassis::arcade(double vertical, double horizontal) {
	pid.setMode(DISABLE); // turns off autonomous task

	// apply thresholding
	vertical = (fabs(vertical) > joystick_threshold ? vertical : 0);
	horizontal = (fabs(horizontal) > joystick_threshold ? horizontal : 0);

	motorMove(leftMotors, vertical + horizontal, false);
	motorMove(rightMotors, vertical - horizontal, false);
}

void Chassis::holonomic(double y, double x, double z) {
	pid.setMode(DISABLE); // turns off autonomous task

	// apply thresholding
	y = (fabs(y) > joystick_threshold ? y : 0);
	x = (fabs(x) > joystick_threshold ? x : 0);
	z = (fabs(z) > joystick_threshold ? z : 0);

	motorMove(frontLeft, y + x + z, false);
	motorMove(frontRight, y - x - z, false);
	motorMove(backLeft, y - x + z, false);
	motorMove(backRight, y + x - z, false);
}

} // namespace arms::chassis

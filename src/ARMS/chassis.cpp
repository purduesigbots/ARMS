#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include "api.h"

using namespace pros;

namespace chassis {

// chassis mode enums
#define LINEAR 1
#define DISABLE 0
#define ANGULAR -1

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

// slew control (autonomous only)
double accel_step; // smaller number = more slew
double arc_step;   // acceleration for arcs
double lastSpeed = 0;

// pid constants
double linearKP;
double linearKD;
double turnKP;
double turnKD;
double arcKP;
double difKP;

/**************************************************/
// edit below with caution!!!
static int mode = DISABLE;
static double linearTarget = 0;
static double turnTarget = 0;
static double vectorAngle = 0;
static int maxSpeed = 100;
bool useVelocity = false;

/**************************************************/
// basic control

// move motor group at given velocity
void motorVoltage(std::shared_ptr<okapi::MotorGroup> motor, int vel) {
	motor->moveVoltage(vel * 120);
}

void motorVelocity(std::shared_ptr<okapi::MotorGroup> motor, int vel) {
	motor->moveVelocity(vel * (double)motor->getGearing() / 200);
}

void motorVoltage(std::shared_ptr<okapi::Motor> motor, int vel) {
	motor->moveVoltage(vel * 120);
}

void motorVelocity(std::shared_ptr<okapi::Motor> motor, int vel) {
	motor->moveVelocity(vel * (double)motor->getGearing() / 200);
}

void setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors->setBrakeMode(b);
	rightMotors->setBrakeMode(b);
	motorVelocity(leftMotors, 0);
	motorVelocity(rightMotors, 0);
}

void reset() {
	lastSpeed = 0;

	motorVelocity(leftMotors, 0);
	motorVelocity(rightMotors, 0);
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

double position(bool yDirection, bool forceEncoder) {
	if (yDirection) {
		double top_pos, bot_pos;

		// TODO change when we add middle encoder
		if (false) {
			top_pos = middleEncoder->get_value();
			bot_pos = middleEncoder->get_value();
		} else {
			top_pos = frontLeft->getPosition() - frontRight->getPosition();
			bot_pos = backRight->getPosition() - backLeft->getPosition();
		}

		return ((mode == ANGULAR ? -top_pos : top_pos) + bot_pos) / 2;

	} else if (imu && mode == ANGULAR && !forceEncoder) {
		// read sensors using IMU if turning and one exists
		return -imu->get_rotation();

	} else {
		double left_pos, right_pos;

		if (leftEncoder) {
			left_pos = leftEncoder->get_value();
			right_pos = rightEncoder->get_value();
		} else {
			left_pos = leftMotors->getPosition();
			right_pos = rightMotors->getPosition();
		}

		return ((mode == ANGULAR ? -left_pos : left_pos) + right_pos) / 2;
	}
}

double difference() {
	double left_pos, right_pos;

	if (leftEncoder) {
		left_pos = leftEncoder->get_value();
		right_pos = rightEncoder->get_value();
	} else {
		left_pos = leftMotors->getPosition();
		right_pos = rightMotors->getPosition();
	}

	return (mode == ANGULAR ? 0 : (left_pos - right_pos));
}

/**************************************************/
// slew control
double slew(double speed) {
	double step;

	if (abs(lastSpeed) < abs(speed))
		if (mode == DISABLE)
			step = arc_step;
		else
			step = accel_step;
	else
		step = 200;

	if (speed > lastSpeed + step)
		lastSpeed += step;
	else if (speed < lastSpeed - step)
		lastSpeed -= step;
	else
		lastSpeed = speed;

	return lastSpeed;
}

/**************************************************/
// chassis settling
bool settled() {
	static int count = 0;
	static double last = 0;
	static double lastTarget = 0;

	double curr = position(false, true);

	double target = turnTarget;
	if (mode == LINEAR)
		target = linearTarget;

	if (abs(last - curr) <
	    (mode == LINEAR ? SETTLE_THRESHOLD_LINEAR : SETTLE_THRESHOLD_ANGULAR))
		count++;
	else
		count = 0;

	if (target != lastTarget)
		count = 0;

	lastTarget = target;
	last = curr;

	// not driving if we haven't moved
	if (count > SETTLE_COUNT)
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
	linearTarget = sp;
	mode = LINEAR;
	vectorAngle = 0;
}

void turnAsync(double sp, int max) {
	mode = ANGULAR;

	if (imu)
		sp += position();
	else
		sp *= degree_constant;

	reset();
	maxSpeed = max;
	turnTarget = sp;
	vectorAngle = 0;
}

void turnAbsoluteAsync(double sp, int max) {
	mode = ANGULAR;

	// convert from absolute to relative set point
	sp = sp - (int)position() % 360;

	// make sure all turns take most efficient route
	if (sp > 180)
		sp -= 360;
	else if (sp < -180)
		sp += 360;

	turnAsync(sp, max);
}

void moveHoloAsync(double distance, double angle, int max) {
	distance *= distance_constant;
	reset();
	maxSpeed = max;
	linearTarget = distance;
	vectorAngle = angle * M_PI / 180;
	mode = 1;
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

void moveHolo(double distance, double angle, int max) {
	moveHoloAsync(distance, angle, max);
	delay(450);
	waitUntilSettled();
}

void fast(double sp, int max) {
	double speed; // speed before PID

	if (sp < 0)
		max = -max;
	reset();
	mode = DISABLE;

	while (abs(position()) < abs(sp * distance_constant)) {
		speed = slew(max);
		// differential PID
		double dif = difference() * difKP;
		if (useVelocity) {
			motorVelocity(leftMotors, speed - dif);
			motorVelocity(rightMotors, speed + dif);
		} else {
			motorVoltage(leftMotors, speed - dif);
			motorVoltage(rightMotors, speed + dif);
		}
		delay(20);
	}
}

void voltage(int t, int left_speed, int right_speed) {
	motorVoltage(leftMotors, left_speed);
	motorVoltage(rightMotors, right_speed == 0 ? left_speed : right_speed);
	delay(t);
}

void velocity(int t, int max) {
	motorVelocity(leftMotors, max);
	motorVelocity(rightMotors, max);
	delay(t);
}

void arc(bool mirror, int arc_length, double rad, int max, int type) {
	reset();
	int time_step = 0;
	mode = DISABLE;
	bool reversed = false;

	// reverse the movement if the length is negative
	if (arc_length < 0) {
		reversed = true;
		arc_length = -arc_length;
	}

	// fix jerk bug between velocity movements
	if (type < 2) {
		motorVelocity(leftMotors, 0);
		motorVelocity(rightMotors, 0);
		delay(10);
	}

	while (time_step < arc_length) {

		// speed
		int error = arc_length - time_step;
		double speed = error * arcKP;

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

		speed = slew(speed); // slew

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
		motorVelocity(leftMotors, mirror ? speed : scaled_speed);
		motorVelocity(rightMotors, mirror ? scaled_speed : speed);

		// increment time step
		time_step += 10;
		delay(10);
	}

	if (type != 1 && type != 2) {
		motorVelocity(leftMotors, 0);
		motorVelocity(rightMotors, 0);
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
int odomTask() {
	double global_x = 0;
	double global_y = 0;
	double heading = M_PI / 2;
	double heading_degrees;
	double prev_heading = heading;

	double prev_left_pos = 0;
	double prev_right_pos = 0;

	double right_arc = 0;
	double left_arc = 0;
	double center_arc = 0;
	double delta_angle = 0;
	double radius = 0;
	double center_displacement = 0;
	double delta_x = 0;
	double delta_y = 0;

	while (true) {
		right_arc = rightMotors->getPosition() - prev_right_pos;
		left_arc = leftMotors->getPosition() - prev_left_pos;
		prev_right_pos = rightMotors->getPosition();
		prev_left_pos = leftMotors->getPosition();
		center_arc = (right_arc + left_arc) / 2.0;

		heading_degrees = imu->get_rotation();
		heading = heading_degrees * M_PI / 180;
		delta_angle = heading - prev_heading;
		prev_heading = heading;

		if (delta_angle != 0) {
			radius = center_arc / delta_angle;
			center_displacement = 2 * sin(delta_angle / 2) * radius;
		} else {
			center_displacement = center_arc;
		}

		delta_x = cos(heading) * center_displacement;
		delta_y = sin(heading) * center_displacement;

		global_x += delta_x;
		global_y += delta_y;

		// printf("%f, %f, %f \n", global_x, global_y, heading);

		delay(10);
	}
}
int chassisTask() {
	double prevError = 0;
	double kp;
	double kd;
	double sp;

	while (1) {
		delay(20);

		if (mode == LINEAR) {
			sp = linearTarget;
			kp = linearKP;
			kd = linearKD;
		} else if (mode == ANGULAR) {
			sp = turnTarget;
			kp = turnKP;
			kd = turnKD;
		} else {
			continue;
		}

		// get position in the x direction
		double sv_x = position();

		// get position in the y direction
		double sv_y = position(true);

		// calculate total displacement using pythagorean theorem
		double sv;
		if (vectorAngle != 0)
			sv = sqrt(pow(sv_x, 2) + pow(sv_y, 2));
		else
			sv = sv_x; // just use the x value for non-holonomic movements

		// speed
		double error = sp - sv;
		double derivative = error - prevError;
		prevError = error;
		double speed = error * kp + derivative * kd;

		// speed limiting
		if (speed > maxSpeed)
			speed = maxSpeed;
		if (speed < -maxSpeed)
			speed = -maxSpeed;

		speed = slew(speed); // slew

		// set motors
		if (vectorAngle != 0) {
			// calculate vectors for each wheel set
			double frontVector = sin(M_PI / 4 - vectorAngle);
			double backVector = sin(M_PI / 4 + vectorAngle);

			// set scaling factor based on largest vector
			double largestVector;
			if (abs(frontVector) > abs(backVector)) {
				largestVector = abs(frontVector);
			} else {
				largestVector = abs(backVector);
			}

			frontVector *= speed / largestVector;
			backVector *= speed / largestVector;

			motorVoltage(frontLeft, frontVector);
			motorVoltage(backLeft, backVector);
			motorVoltage(frontRight, backVector);
			motorVoltage(backRight, frontVector);

		} else {
			double dif = difference() * difKP;

			printf("proportional %.2f, derivative %.2f, speed %.2f, dif %.2f\n",
			       error * kp, derivative * kd, speed, dif);

			if (useVelocity) {
				motorVelocity(leftMotors, (speed - dif) * mode);
				motorVelocity(rightMotors, speed + dif);
			} else {
				motorVoltage(leftMotors, (speed - dif) * mode);
				motorVoltage(rightMotors, speed + dif);
			}
		}
	}
}

void startTasks() {
	Task chassis_task(chassisTask);
	if (imu) {
		Task odom_task(odomTask);
	}
}

std::shared_ptr<ADIEncoder> initEncoder(int encoderPort, int expanderPort) {
	std::shared_ptr<ADIEncoder> encoder;

	int encoderPort2 =
	    abs((encoderPort > 0) ? (abs(encoderPort) + 1) : encoderPort--);
	encoderPort = abs(encoderPort);

	if (expanderPort != 0) {
		std::tuple<int, int, int> pair(expanderPort, encoderPort, encoderPort2);
		encoder = std::make_shared<ADIEncoder>(pair, false);
	} else {
		encoder = std::make_shared<ADIEncoder>(encoderPort, encoderPort2);
	}

	return encoder;
}

void init(std::initializer_list<okapi::Motor> leftMotors,
          std::initializer_list<okapi::Motor> rightMotors, int gearset,
          double distance_constant, double degree_constant, double accel_step,
          double arc_step, double linearKP, double linearKD, double turnKP,
          double turnKD, double arcKP, double difKP, int imuPort,
          std::tuple<int, int, int> encoderPorts, int expanderPort) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::accel_step = accel_step;
	chassis::arc_step = arc_step;
	chassis::linearKP = linearKP;
	chassis::linearKD = linearKD;
	chassis::turnKP = turnKP;
	chassis::turnKD = turnKD;
	chassis::arcKP = arcKP;
	chassis::difKP = difKP;

	// configure chassis motors
	chassis::leftMotors = std::make_shared<okapi::MotorGroup>(leftMotors);
	chassis::rightMotors = std::make_shared<okapi::MotorGroup>(rightMotors);
	chassis::leftMotors->setGearing((okapi::AbstractMotor::gearset)gearset);
	chassis::rightMotors->setGearing((okapi::AbstractMotor::gearset)gearset);

	// initialize imu
	if (imuPort != 0) {
		imu = std::make_shared<Imu>(imuPort);
		imu->reset();
		while (imu->is_calibrating()) {
			delay(10);
		}
		delay(1000);
		printf("IMU calibrated!");
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

	// start task
	startTasks();
}

/**************************************************/
// operator control
void tank(int left_speed, int right_speed) {
	mode = DISABLE; // turns off autonomous tasks
	motorVoltage(leftMotors, left_speed);
	motorVoltage(rightMotors, right_speed);
}

void arcade(int vertical, int horizontal) {
	mode = DISABLE; // turns off autonomous task
	motorVoltage(leftMotors, vertical + horizontal);
	motorVoltage(rightMotors, vertical - horizontal);
}

void holonomic(int x, int y, int z) {
	mode = 0; // turns off autonomous task
	motorVoltage(frontLeft, x + y + z);
	motorVoltage(frontRight, x - y - z);
	motorVoltage(backLeft, x + y - z);
	motorVoltage(backRight, x - y + z);
}

} // namespace chassis

#include "greenhat/drive.h"
#include "api.h"
#include "greenhat/config.h"
using namespace pros;

namespace greenhat {

// drive motors
okapi::MotorGroup leftMotors = {LEFT_MOTORS};
okapi::MotorGroup rightMotors = {RIGHT_MOTORS};
okapi::AbstractMotor::gearset gearset = okapi::AbstractMotor::gearset::GEARSET;

// gyro (set to 0 if not using)
int gyro_port = GYRO_PORT;
Imu iSens(gyro_port);

// distance constants
const int distance_constant = DISTANCE_CONSTANT; // ticks per foot
const double degree_constant = DEGREE_CONSTANT;  // ticks per degree

// slew control (autonomous only)
const int accel_step = ACCEL_STEP;   // smaller number = more slew
const int deccel_step = DECCEL_STEP; // 200 = no slew
const int arc_step = ARC_STEP;       // acceleration for arcs

// pid constants
const double driveKP = DRIVE_KP;
const double driveKD = DRIVE_KD;
const double turnKP = TURN_KP;
const double turnKD = TURN_KD;
const double arcKP = ARC_KP;

/**************************************************/
// edit below with caution!!!
static int driveMode = 0;
static int driveTarget = 0;
static int turnTarget = 0;
static int maxSpeed = 100;

/**************************************************/
// basic control
void left_drive(int vel) {
	vel *= 120;
	leftMotors.moveVoltage(vel);
}

void right_drive(int vel) {
	vel *= 120;
	rightMotors.moveVoltage(vel);
}

void left_drive_vel(int vel) {
	vel *= (double)gearset / 100;
	leftMotors.moveVelocity(vel);
}

void right_drive_vel(int vel) {
	vel *= (double)gearset / 100;
	rightMotors.moveVelocity(vel);
}

void setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors.setBrakeMode(b);
	rightMotors.setBrakeMode(b);
	leftMotors.moveVelocity(0);
	rightMotors.moveVelocity(0);
}

void reset() {
	leftMotors.tarePosition();
	rightMotors.tarePosition();
	setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

int drivePos() {
	return (rightMotors.getPosition() + leftMotors.getPosition()) / 2;
}

/**************************************************/
// slew control
static int lastSpeed = 0;
int slew(int speed) {
	int step;

	if (abs(lastSpeed) < abs(speed))
		if (driveMode == 0)
			step = arc_step;
		else
			step = accel_step;
	else
		step = deccel_step;

	if (speed > lastSpeed + step)
		lastSpeed += step;
	else if (speed < lastSpeed - step)
		lastSpeed -= step;
	else {
		lastSpeed = speed;
	}

	return lastSpeed;
}

/**************************************************/
// drive settling
bool isDriving() {
	static int count = 0;
	static int last = 0;
	static int lastTarget = 0;

	int curr = drivePos();

	int target = turnTarget;
	if (driveMode == 1)
		target = driveTarget;

	if (abs(last - curr) < 3)
		count++;
	else
		count = 0;

	if (target != lastTarget)
		count = 0;

	lastTarget = target;
	last = curr;

	// not driving if we haven't moved
	if (count > 4)
		return false;
	else
		return true;
}

void waitUntilSettled() {
	while (isDriving())
		delay(10);
}

/**************************************************/
// autonomous functions
void driveAsync(double sp, int max) {
	sp *= distance_constant;
	reset();
	maxSpeed = max;
	driveTarget = sp;
	driveMode = 1;
}

void turnAsync(double sp, int max) {
	sp *= degree_constant;
	reset();
	maxSpeed = max;
	turnTarget = sp;
	driveMode = -1;
}

void drive(double sp, int max) {
	driveAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void turn(double sp, int max) {
	turnAsync(sp, max);
	delay(450);
	waitUntilSettled();
}

void fastDrive(double sp, int max) {
	if (sp < 0)
		max = -max;
	reset();
	lastSpeed = max;
	driveMode = 0;
	left_drive(max);
	right_drive(max);

	if (sp > 0)
		while (drivePos() < sp * distance_constant)
			delay(20);
	else
		while (drivePos() > sp * distance_constant)
			delay(20);
}

void timeDrive(int t, int left, int right) {
	left_drive(left);
	right_drive(right == 0 ? left : right);
	delay(t);
}

void velocityDrive(int t, int max) {
	left_drive_vel(max);
	right_drive_vel(max);
	delay(t);
}

void arc(bool mirror, int arc_length, double rad, int max, int type) {
	reset();
	int time_step = 0;
	driveMode = 0;
	bool reversed = false;

	// reverse the movement if the length is negative
	if (arc_length < 0) {
		reversed = true;
		arc_length = -arc_length;
	}

	// fix jerk bug between velocity movements
	if (type != 2) {
		leftMotors.moveVelocity(0);
		rightMotors.moveVelocity(0);
		delay(10);
	}

	while (time_step < arc_length) {

		// speed
		int error = arc_length - time_step;
		int speed = error * arcKP;

		if (type == 1)
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
			scaled_speed = speed * (double)time_step / arc_length;
		else if (type == 2)
			scaled_speed = speed * (1 - (double)time_step / arc_length);

		// assign drive motor speeds
		left_drive_vel(mirror ? speed : scaled_speed);
		right_drive_vel(mirror ? scaled_speed : speed);

		// increment time step
		time_step += 10;
		delay(10);
	}

	if (type != 1) {
		leftMotors.moveVelocity(0);
		rightMotors.moveVelocity(0);
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
	velocityDrive(mid, max);

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
int driveTask() {
	int prevError = 0;
	double kp;
	double kd;
	int sp;

	while (1) {
		delay(20);

		if (driveMode == 1) {
			sp = driveTarget;
			kp = driveKP;
			kd = driveKD;
		} else if (driveMode == -1) {
			sp = turnTarget;
			kp = turnKP;
			kd = turnKD;
		} else {
			continue;
		}

		// read sensors
		int sv =
		    (rightMotors.getPosition() + leftMotors.getPosition() * driveMode) / 2;
		if (gyro_port != 0 && driveMode == -1) {
			sv = -iSens.get_rotation();
		}

		// speed
		int error = sp - sv;
		int derivative = error - prevError;
		prevError = error;
		int speed = error * kp + derivative * kd;

		// speed limiting
		if (speed > maxSpeed)
			speed = maxSpeed;
		if (speed < -maxSpeed)
			speed = -maxSpeed;

		speed = slew(speed); // slew

		// set motors
		left_drive(speed * driveMode);
		right_drive(speed);
	}
}

void initDrive() {
	Task drive_task(driveTask);
	if (gyro_port != 0) {
		while (iSens.is_calibrating())
			delay(20);
	}
	leftMotors.setGearing(gearset);
	rightMotors.setGearing(gearset);
	leftMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
	rightMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
}

/**************************************************/
// operator control
void tank(int left, int right) {
	driveMode = 0; // turns off autonomous tasks
	left_drive(left);
	right_drive(right);
}

void arcade(int vertical, int horizontal) {
	driveMode = 0; // turns off autonomous task
	left_drive(vertical + horizontal);
	right_drive(vertical - horizontal);
}

} // namespace greenhat

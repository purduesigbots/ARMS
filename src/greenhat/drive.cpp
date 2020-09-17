#include "greenhat/drive.h"
#include "api.h"
#include "greenhat/config.h"
using namespace pros;

namespace greenhat {

// drive motor groups
std::shared_ptr<okapi::MotorGroup> leftMotors;
std::shared_ptr<okapi::MotorGroup> rightMotors;

// individual drive motors
std::shared_ptr<okapi::Motor> frontLeft;
std::shared_ptr<okapi::Motor> frontRight;
std::shared_ptr<okapi::Motor> backLeft;
std::shared_ptr<okapi::Motor> backRight;

// distance constants
int distance_constant;  // ticks per foot
double degree_constant; // ticks per degree

// slew control (autonomous only)
int accel_step;  // smaller number = more slew
int deccel_step; // 200 = no slew
int arc_step;    // acceleration for arcs

// pid constants
double driveKP;
double driveKD;
double turnKP;
double turnKD;
double arcKP;

/**************************************************/
// edit below with caution!!!
static int driveMode = 0;
static double driveTarget = 0;
static double vectorAngle = 0;
static double turnTarget = 0;
static int maxSpeed = 100;

/**************************************************/
// basic control
void left_drive(int vel) {
	vel *= 120;
	leftMotors->moveVoltage(vel);
}

void right_drive(int vel) {
	vel *= 120;
	rightMotors->moveVoltage(vel);
}

void left_drive_vel(int vel) {
	vel *= (double)leftMotors->getGearing() / 100;
	leftMotors->moveVelocity(vel);
}

void right_drive_vel(int vel) {
	vel *= (double)leftMotors->getGearing() / 100;
	rightMotors->moveVelocity(vel);
}

// individual motors
void fl(int vel){
	vel *= 120;
	frontLeft->moveVoltage(vel);
}

void fr(int vel){
	vel *= 120;
	frontRight->moveVoltage(vel);
}

void bl(int vel){
	vel *= 120;
	backLeft->moveVoltage(vel);
}

void br(int vel){
	vel *= 120;
	backRight->moveVoltage(vel);
}


void setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors->setBrakeMode(b);
	rightMotors->setBrakeMode(b);
	left_drive_vel(0);
	right_drive_vel(0);
}

void reset() {
	leftMotors->tarePosition();
	rightMotors->tarePosition();

	frontLeft->tarePosition();
	frontRight->tarePosition();
	backLeft->tarePosition();
	backRight->tarePosition();
}

int drivePos() {
	return (rightMotors->getPosition() + leftMotors->getPosition()) / 2;
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
	vectorAngle = 0;
	driveMode = 1;
}

void turnAsync(double sp, int max) {
	sp *= degree_constant;
	reset();
	maxSpeed = max;
	turnTarget = sp;
	vectorAngle = 0;
	driveMode = -1;
}

void driveHoloAsync(double distance, double angle, int max){
	distance *= distance_constant;
	angle *= degree_constant;
	reset();
	maxSpeed = max;
	driveTarget = distance;
	vectorAngle = angle*M_PI/180;
	driveMode = 1;
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

void driveHolo(double distance, double angle, int max){
	driveHoloAsync(distance, angle, max);
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
	if (type < 2) {
		left_drive_vel(0);
		right_drive_vel(0);
		delay(10);
	}

	while (time_step < arc_length) {

		// speed
		int error = arc_length - time_step;
		int speed = error * arcKP;

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
			scaled_speed *= std::abs(2*(.5-(double)time_step/arc_length));
		else if(type == 3)
			scaled_speed *= (1 - (double)time_step / arc_length);


		// assign drive motor speeds
		left_drive_vel(mirror ? speed : scaled_speed);
		right_drive_vel(mirror ? scaled_speed : speed);

		// increment time step
		time_step += 10;
		delay(10);
	}

	if (type != 1 && type != 2) {
		left_drive_vel(0);
		right_drive_vel(0);
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

		// get position in the x direction
		int sv_x =
		    (frontLeft->getPosition() + backLeft->getPosition() +
				(frontRight->getPosition() + backRight->getPosition()) * driveMode) /
		    4;

		// get position in the y direction
		int sv_y =
				(frontLeft->getPosition() - backLeft->getPosition() -
				frontRight->getPosition() + backRight->getPosition()) /
				4;


		// calculate total displacement using pythagorean theorem
		int sv;
		if(vectorAngle != 0){
			sv = sqrt(pow(sv_x, 2) + pow(sv_y, 2));
		}else{
			sv = sv_x; // just use the x value for non-holonomic movements
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
		if(vectorAngle != 0){
			// calculate vectors for each wheel set
			double frontVector = sin(M_PI/4 - vectorAngle);
			double backVector = sin(M_PI/4 + vectorAngle);

			// set scaling factor based on largest vector
			double largestVector;
			if(abs(frontVector) > abs(backVector)){
				largestVector = abs(frontVector);
			}else{
				largestVector = abs(backVector);
			}

			frontVector *= speed / largestVector;
			backVector *= speed / largestVector;

			fl(frontVector);
			bl(backVector);
			fr(backVector);
			br(frontVector);

		}else{
			left_drive(speed * driveMode);
			right_drive(speed);
		}
	}
}

void startTask() {
	Task drive_task(driveTask);
}

void initDrive(std::initializer_list<okapi::Motor> leftMotors,
               std::initializer_list<okapi::Motor> rightMotors, int gearset,
               int distance_constant, double degree_constant, int accel_step,
               int deccel_step, int arc_step, double driveKP, double driveKD,
               double turnKP, double turnKD, double arcKP) {

	// assign constants
	greenhat::distance_constant = distance_constant;
	greenhat::degree_constant = degree_constant;
	greenhat::accel_step = accel_step;
	greenhat::deccel_step = deccel_step;
	greenhat::arc_step = arc_step;
	greenhat::driveKP = driveKP;
	greenhat::driveKD = driveKD;
	greenhat::turnKP = turnKP;
	greenhat::turnKD = turnKD;
	greenhat::arcKP = arcKP;

	// configure motor groups
	greenhat::leftMotors = std::make_shared<okapi::MotorGroup>(leftMotors);
	greenhat::rightMotors = std::make_shared<okapi::MotorGroup>(rightMotors);
	greenhat::leftMotors->setGearing((okapi::AbstractMotor::gearset)gearset);
	greenhat::rightMotors->setGearing((okapi::AbstractMotor::gearset)gearset);

	// configure individual motors for holonomic drives
	greenhat::frontLeft = std::make_shared<okapi::Motor>(*leftMotors.begin());
	greenhat::backLeft = std::make_shared<okapi::Motor>(*leftMotors.end());
	greenhat::frontRight = std::make_shared<okapi::Motor>(*leftMotors.begin());
	greenhat::backRight = std::make_shared<okapi::Motor>(*leftMotors.end());

	// set gearing for individual motors
	greenhat::frontLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	greenhat::frontLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	greenhat::frontLeft->setGearing((okapi::AbstractMotor::gearset)gearset);
	greenhat::frontLeft->setGearing((okapi::AbstractMotor::gearset)gearset);

	// start task
	Task drive_task(driveTask);
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

void holonomic(int x, int y, int z){
	driveMode = 0; // turns off autonomous task
	fl(x+y+z);
	fr(-x+y+z);
	bl(x-y+z);
	br(-x-y+z);
}

} // namespace greenhat

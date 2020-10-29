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

// quad encoders
std::shared_ptr<ADIEncoder> leftEncoder;
std::shared_ptr<ADIEncoder> rightEncoder;

// distance constants
int distance_constant;  // ticks per foot
double degree_constant; // ticks per degree

// slew control (autonomous only)
int accel_step;  // smaller number = more slew
int deccel_step; // 200 = no slew
int arc_step;    // acceleration for arcs

// pid constants
double linearKP;
double linearKD;
double turnKP;
double turnKD;
double arcKP;

/**************************************************/
// edit below with caution!!!
static int mode = DISABLE;
static int linearTarget = 0;
static int turnTarget = 0;
static int maxSpeed = 100;

/**************************************************/
// basic control
void left(int vel) {
	vel *= 120;
	leftMotors->moveVoltage(vel);
}

void right(int vel) {
	vel *= 120;
	rightMotors->moveVoltage(vel);
}

void left_vel(int vel) {
	vel *= (double)leftMotors->getGearing() / 100;
	leftMotors->moveVelocity(vel);
}

void right_vel(int vel) {
	vel *= (double)leftMotors->getGearing() / 100;
	rightMotors->moveVelocity(vel);
}

void setBrakeMode(okapi::AbstractMotor::brakeMode b) {
	leftMotors->setBrakeMode(b);
	rightMotors->setBrakeMode(b);
	left_vel(0);
	right_vel(0);
}

void reset() {
	leftMotors->tarePosition();
	rightMotors->tarePosition();
}

int position() {
	int left_pos;
	if (leftEncoder != NULL) {
		left_pos = leftEncoder->getPosition();
		return ((mode == ANGULAR ? -left_pos : left_pos) +
                        (rightEncoder->getPosition()));
	}
	left_pos = leftMotors->getPosition();
	return ((mode == ANGULAR ? -left_pos : left_pos) +
                (rightMotors->getPosition()));
}

/**************************************************/
// slew control
static int lastSpeed = 0;
int slew(int speed) {
	int step;

	if (abs(lastSpeed) < abs(speed))
		if (mode == DISABLE)
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
// chassis settling
bool isDriving() {
	static int count = 0;
	static int last = 0;
	static int lastTarget = 0;

	int curr = position();

	int target = turnTarget;
	if (mode == ANGULAR)
		target = linearTarget;

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
void moveAsync(double sp, int max) {
	sp *= distance_constant;
	reset();
	maxSpeed = max;
	linearTarget = sp;
	mode = LINEAR;
}

void turnAsync(double sp, int max) {
	sp *= degree_constant;
	reset();
	maxSpeed = max;
	turnTarget = sp;
	mode = ANGULAR;
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

void fast(double sp, int max) {
	if (sp < 0)
		max = -max;
	reset();
	lastSpeed = max;
	mode = DISABLE;
	left(max);
	right(max);

	if (sp > 0)
		while (position() < sp * distance_constant)
			delay(20);
	else
		while (position() > sp * distance_constant)
			delay(20);
}

void time(int t, int left_speed, int right_speed) {
	left(left_speed);
	right(right_speed == 0 ? left_speed : right_speed);
	delay(t);
}

void velocity(int t, int max) {
	left_vel(max);
	right_vel(max);
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
		left_vel(0);
		right_vel(0);
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
			scaled_speed *= std::abs(2 * (.5 - (double)time_step / arc_length));
		else if (type == 3)
			scaled_speed *= (1 - (double)time_step / arc_length);

		// assign chassis motor speeds
		left_vel(mirror ? speed : scaled_speed);
		right_vel(mirror ? scaled_speed : speed);

		// increment time step
		time_step += 10;
		delay(10);
	}

	if (type != 1 && type != 2) {
		left_vel(0);
		right_vel(0);
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

		printf("%f, %f, %f \n", global_x, global_y, heading);

		delay(10);
	}
}
int chassisTask() {
	int prevError = 0;
	double kp;
	double kd;
	int sp;

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

		// read sensors
		int sv = position();

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
		left(speed * mode);
		right(speed);
	}
}

void startTasks() {
	Task chassis_task(chassisTask);
	if(imu){
		Task odom_task(odomTask);
	}
}

void init(std::initializer_list<okapi::Motor> leftMotors,
          std::initializer_list<okapi::Motor> rightMotors, int gearset,
          int distance_constant, double degree_constant, int accel_step,
          int deccel_step, int arc_step, double linearKP, double linearKD,
          double turnKP, double turnKD, double arcKP, int imuPort,
          std::tuple<int, int, int, int> encoderPorts) {

	// assign constants
	chassis::distance_constant = distance_constant;
	chassis::degree_constant = degree_constant;
	chassis::accel_step = accel_step;
	chassis::deccel_step = deccel_step;
	chassis::arc_step = arc_step;
	chassis::linearKP = linearKP;
	chassis::linearKD = linearKD;
	chassis::turnKP = turnKP;
	chassis::turnKD = turnKD;
	chassis::arcKP = arcKP;

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
		printf("IMU calibrated!");
	}

	if (std::get<0>(encoderPorts) != 0) {
		leftEncoder = std::make_shared<ADIEncoder>(std::get<0>(encoderPorts),
		                                           std::get<1>(encoderPorts));
		rightEncoder = std::make_shared<ADIEncoder>(std::get<2>(encoderPorts),
		                                            std::get<3>(encoderPorts));
	}

	// start task
	startTasks();
}

/**************************************************/
// operator control
void tank(int left_speed, int right_speed) {
	mode = DISABLE; // turns off autonomous tasks
	left(left_speed);
	right(right_speed);
}

void arcade(int vertical, int horizontal) {
	mode = DISABLE; // turns off autonomous task
	left(vertical + horizontal);
	right(vertical - horizontal);
}

} // namespace chassis

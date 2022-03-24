#include "ARMS/lib.h"
#include "api.h"

namespace arms::odom {

// sensors
std::shared_ptr<okapi::ContinuousRotarySensor> leftEncoder;
std::shared_ptr<okapi::ContinuousRotarySensor> rightEncoder;
std::shared_ptr<okapi::ContinuousRotarySensor> middleEncoder;
std::shared_ptr<pros::Imu> imu;

// output the odometry data to the terminal
bool debug;

// tracker wheel configuration
double left_right_distance;
double middle_distance;
double left_right_tpi;
double middle_tpi;

// odom position values
Point position;
double heading;

// previous values
double prev_left_pos = 0;
double prev_right_pos = 0;
double prev_middle_pos = 0;
double prev_heading = 0;

bool reverse;

int odomTask() {

	position.x = 0;
	position.y = 0;
	heading = 0;

	while (true) {

		// get positions of each encoder
		double left_pos = leftEncoder->get();
		double right_pos = rightEncoder->get();
		double middle_pos = middleEncoder ? middleEncoder->get() : 0;

		// calculate change in each encoder
		double delta_left = (left_pos - prev_left_pos) / left_right_tpi;
		double delta_right = (right_pos - prev_right_pos) / left_right_tpi;
		double delta_middle =
		    middleEncoder ? (middle_pos - prev_middle_pos) / middle_tpi : 0;

		// calculate new heading
		double delta_angle;
		if (imu) {
			heading = -imu->get_rotation() * M_PI / 180.0;
			delta_angle = heading - prev_heading;
		} else {
			delta_angle = (delta_right - delta_left) / (left_right_distance * 2);

			heading += delta_angle;
		}

		// store previous positions
		prev_left_pos = left_pos;
		prev_right_pos = right_pos;
		prev_middle_pos = middle_pos;
		prev_heading = heading;

		// calculate local displacement
		double local_x;
		double local_y;

		if (delta_angle) {
			double i = sin(delta_angle / 2.0) * 2.0;
			local_x = (delta_right / delta_angle + left_right_distance) * i;
			local_y = (delta_middle / delta_angle + middle_distance) * i;
		} else {
			local_x = delta_right;
			local_y = delta_middle;
		}

		// If we want the robot to move in reverse, then we need to flip the heading of the robot
		// 180 degrees
		double rHeading = heading - (reverse ? M_PI : 0.0);
		double p = rHeading - delta_angle / 2.0; // global angle

		// convert to absolute displacement
		position.x += cos(p) * local_x + sin(p) * local_y;
		position.y += cos(p) * local_y + sin(p) * local_x;

		if (debug)
			printf("%.2f, %.2f, %.2f \n", position.x, position.y, getHeading());

		pros::delay(10);
	}
}

void reset(Point point) {
	position.x = point.x;
	position.y = point.y;
	chassis::virtualPosition = position;
}

void reset(Point point, double angle) {
	reset(point);
	heading = angle * M_PI / 180.0;
	prev_heading = heading;
	imu->set_heading(-angle);
}

Point getPosition() {
	return position;
}

double getHeading(bool radians) {
	if (radians)
		return heading;
	return heading * 180 / M_PI;
}

double getAngleError(Point point) {
	double x = point.x;
	double y = point.y;

	x -= position.x;
	y -= position.y;

	double delta_theta = atan2(y, x) - heading;

	// if movement is reversed, calculate delta_theta using a 180 degree rotation
	// of the target point
	if (pid::reverse) {
		delta_theta = atan2(-y, -x) - heading;
	}

	while (fabs(delta_theta) > M_PI) {
		delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
	}

	return delta_theta;
}

double getDistanceError(Point point) {
	double x = point.x;
	double y = point.y;

	y -= position.y;
	x -= position.x;
	return sqrt(x * x + y * y);
}

std::shared_ptr<okapi::ContinuousRotarySensor> initEncoder(int p1, int exp,
                                                           int type) {
	if (type == ENCODER_ROTATION)
		return std::make_shared<okapi::RotationSensor>(p1, p1 < 0);
	if (exp != 0) {
		std::tuple<int, int, int> pair(exp, abs(p1), abs(p1 + 1));
		return std::make_shared<okapi::ADIEncoder>(pair, p1 < 0);
	} else
		return std::make_shared<okapi::ADIEncoder>(abs(p1), abs(p1) + 1, p1 < 0);
}

void init(bool debug, int encoderType, std::array<int, 3> encoderPorts,
          int expanderPort, int imuPort, double left_right_distance,
          double middle_distance, double left_right_tpi, double middle_tpi) {
	odom::debug = debug;
	odom::left_right_distance = left_right_distance;
	odom::middle_distance = middle_distance;
	odom::left_right_tpi = left_right_tpi;
	odom::middle_tpi = middle_tpi;

	// encoders
	if (encoderPorts[0] != 0) {
		leftEncoder = initEncoder(encoderPorts[0], expanderPort, encoderType);
		rightEncoder = initEncoder(encoderPorts[1], expanderPort, encoderType);
	} else {
		leftEncoder = chassis::leftMotors->getEncoder();
		rightEncoder = chassis::rightMotors->getEncoder();
	}
	if (encoderPorts[2] != 0)
		middleEncoder = initEncoder(encoderPorts[2], expanderPort, encoderType);

	pros::Task odom_task(odomTask);

	// initialize imu
	if (imuPort != 0) {
		imu = std::make_shared<pros::Imu>(imuPort);
		imu->reset();
		pros::delay(2000); // wait for IMU intialization
	}
	pros::delay(100);
	reset();
}

} // namespace arms::odom

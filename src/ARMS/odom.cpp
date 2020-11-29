#include "ARMS/odom.h"
#include "ARMS/chassis.h"
#include "api.h"

using namespace pros;

namespace odom {

bool debug;
double chassis_width;
double exit_error;

// odom tracking values
double global_x;
double global_y;
double heading;
double prev_heading = 0;
double prev_left_pos = 0;
double prev_right_pos = 0;
double prev_middle_pos = 0;

int odomTask() {

	global_x = 0;
	global_y = 0;

	while (true) {
		int left_pos;
		int right_pos;

		if (chassis::leftEncoder) {
			left_pos = chassis::leftEncoder->get_value();
			right_pos = chassis::rightEncoder->get_value();
		} else {
			left_pos = chassis::leftMotors->getPosition();
			right_pos = chassis::rightMotors->getPosition();
		}

		double left_arc = left_pos - prev_left_pos;
		double right_arc = right_pos - prev_right_pos;

		prev_left_pos = left_pos;
		prev_right_pos = right_pos;

		double center_arc = (right_arc + left_arc) / 2.0;

		int horizontal_val = 0;
		if (chassis::middleEncoder)
			horizontal_val = chassis::middleEncoder->get_value();

		double horizontal_arc = horizontal_val - prev_middle_pos;

		prev_middle_pos = horizontal_val;

		double heading_degrees;

		if (chassis::imu) {
			heading_degrees = chassis::imu->get_rotation();
			heading = heading_degrees * M_PI / 180;
		} else {
			heading = prev_heading + (left_arc - right_arc) /
			                             (chassis::distance_constant * chassis_width);
			heading_degrees = heading * 180 / M_PI;
		}

		double delta_angle = heading - prev_heading;
		prev_heading = heading;

		double center_displacement;
		double horizontal_displacement;
		if (delta_angle != 0) {
			center_displacement =
			    2 * sin(delta_angle / 2) * (center_arc / delta_angle);
			horizontal_displacement =
			    2 * sin(delta_angle / 2) * (horizontal_arc / delta_angle);
		} else {
			center_displacement = center_arc;
			horizontal_displacement = horizontal_arc;
		}

		double delta_x = cos(heading) * center_displacement +
		                 sin(heading) * horizontal_displacement;
		double delta_y = sin(heading) * center_displacement +
		                 cos(heading) * horizontal_displacement;

		global_x += delta_x / chassis::distance_constant;
		global_y += delta_y / chassis::distance_constant;

		if (debug)
			printf("%.2f, %.2f, %.2f \n", global_x, global_y, heading_degrees);

		delay(10);
	}
}

double getAngleError(std::array<double, 2> point) {
	double x = point[0];
	double y = point[1];

	x -= global_x;
	y -= global_y;

	double delta_theta = heading - atan2(y, x);

	while (fabs(delta_theta) > M_PI) {
		delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
	}

	return delta_theta;
}

double getDistanceError(std::array<double, 2> point) {
	double x = point[0];
	double y = point[1];

	x -= global_x;
	y -= global_y;
	return sqrt(x * x + y * y);
}

void goToPointAsync(std::array<double, 2> point, double max) {
	chassis::reset();
	chassis::maxSpeed = max;
}

void goToPoint(std::array<double, 2> point, double max) {
	goToPointAsync(point, max);
	delay(450);
	chassis::waitUntilSettled();
}

void init(bool debug, double chassis_width, double exit_error) {
	odom::debug = debug;
	odom::chassis_width = chassis_width;
	odom::exit_error = exit_error;
}

} // namespace odom

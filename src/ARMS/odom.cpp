#include "ARMS/odom.h"
#include "ARMS/chassis.h"
#include "api.h"

using namespace pros;

namespace odom {

bool debug;
double chassis_width;
double slew_step;
double exit_error;

// pid constants
double linearKP;
double linearKD;
double angularKP;
double angularKD;

// odom tracking values
double global_x;
double global_y;
double heading;

int odomTask() {
	double prev_heading = 0;
	double prev_left_pos = 0;
	double prev_right_pos = 0;
	double prev_middle_pos = 0;

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

void goToPoint(std::array<double, 2> point) {
	double max_speed = 80; // 100 max

	double left_prev = 0;
	double right_prev = 0;

	double vel_prev_error = getDistanceError(point);
	double ang_prev_error = getAngleError(point);

	bool driving = true;

	while (driving) {
		int reverse = 1;

		double lin_error = getDistanceError(point); // linear
		double ang_error = getAngleError(point);    // angular

		if (fabs(ang_error) > M_PI_2) {
			ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
			reverse = -1;
		}

		double vel_derivative = lin_error - vel_prev_error;
		double ang_derivative = ang_error - ang_prev_error;

		vel_prev_error = lin_error;
		ang_prev_error = ang_error;

		double forward_speed = linearKP * lin_error + linearKD * vel_derivative;
		double turn_modifier = angularKP * ang_error + angularKD * ang_derivative;

		forward_speed *= reverse;

		double left_speed = forward_speed - turn_modifier;
		double right_speed = forward_speed + turn_modifier;

		if (left_speed > max_speed) {
			double diff = left_speed - max_speed;
			left_speed -= diff;
			right_speed -= diff;
		} else if (left_speed < -max_speed) {
			double diff = left_speed + max_speed;
			left_speed -= diff;
			right_speed -= diff;
		}

		if (right_speed > max_speed) {
			double diff = right_speed - max_speed;
			left_speed -= diff;
			right_speed -= diff;
		} else if (right_speed < -max_speed) {
			double diff = right_speed + max_speed;
			left_speed -= diff;
			right_speed -= diff;
		}

		left_speed = chassis::slew(left_speed, chassis::accel_step, &left_prev);
		right_speed = chassis::slew(right_speed, chassis::accel_step, &right_prev);

		left_prev = left_speed;
		right_prev = right_speed;

		chassis::leftMotors->moveVoltage(left_speed * 120);
		chassis::rightMotors->moveVoltage(right_speed * 120);

		if (lin_error < exit_error)
			driving = false;
		delay(20);
	}
	chassis::leftMotors->moveVoltage(0);
	chassis::rightMotors->moveVoltage(0);
}

void init(bool debug, double chassis_width, double linearKP, double linearKD,
          double angularKP, double angularKD, double slew_step,
          double exit_error) {

	odom::debug = debug;
	odom::chassis_width = chassis_width;
	odom::linearKP = linearKP;
	odom::linearKD = linearKD;
	odom::angularKP = angularKP;
	odom::angularKD = angularKD;
	odom::slew_step = slew_step;
	odom::exit_error = exit_error;
}

} // namespace odom

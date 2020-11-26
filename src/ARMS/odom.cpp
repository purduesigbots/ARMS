#include "ARMS/odom.h"
#include "ARMS/chassis.h"
#include "api.h"

using namespace pros;

namespace odom {

double global_x;
double global_y;
double heading;
double heading_degrees;

int odomTask() {
	double drive_constant = 47.94; // ticks per inch

	double prev_heading = 0;
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

	global_x = 0;
	global_y = 0;

	while (true) {
		right_arc = chassis::rightMotors->getPosition() - prev_right_pos;
		left_arc = chassis::leftMotors->getPosition() - prev_left_pos;
		prev_right_pos = chassis::rightMotors->getPosition();
		prev_left_pos = chassis::leftMotors->getPosition();
		center_arc = (right_arc + left_arc) / 2.0;

		heading_degrees = chassis::imu->get_rotation();
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

		global_x += delta_x / drive_constant;
		global_y += delta_y / drive_constant;

		printf("%f, %f, %f \n", global_x, global_y, heading_degrees);

		delay(10);
	}
}

double getAngle(std::array<double, 2> point) {
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

double getDistance(std::array<double, 2> point) {
	double x = point[0];
	double y = point[1];

	x -= global_x;
	y -= global_y;
	return sqrt(x * x + y * y);
}

void goToPoint(std::array<double, 2> point) {
	double max_speed = 80; // 100 max
	double exit_error = 5; // radius around point in inches

	double kP_vel = 8.0;
	double kI_vel = 0.0;
	double kD_vel = 0.0;

	double kP_ang = 50.0;
	double kI_ang = 0.0;
	double kD_ang = 0.0;

	double slew_step = 10;
	double left_prev = 0;
	double right_prev = 0;

	double vel_prev_error = getDistance(point);
	double ang_prev_error = getAngle(point);

	bool driving = true;

	while (driving) {
		int reverse = 1;

		double vel_error = getDistance(point);
		double ang_error = getAngle(point);

		if (fabs(ang_error) > M_PI_2) {
			ang_error = ang_error - (ang_error / fabs(ang_error)) * M_PI;
			reverse = -1;
		}

		double vel_derivative = vel_error - vel_prev_error;
		double ang_derivative = ang_error - ang_prev_error;

		vel_prev_error = vel_error;
		ang_prev_error = ang_error;

		double forward_speed = kP_vel * vel_error + kD_vel * vel_derivative;
		double turn_modifier = kP_ang * ang_error + kD_ang * ang_derivative;

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

		if (vel_error < exit_error)
			driving = false;
		delay(20);
	}
	chassis::leftMotors->moveVoltage(0);
	chassis::rightMotors->moveVoltage(0);
}

} // namespace odom

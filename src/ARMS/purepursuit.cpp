#include "ARMS/odom.h"

using namespace pros;

namespace arms::odom {

bool last_segment = false;
double m, a, b, c;
double x1, x2, x3, y1, y2, y3;
double x_int_1, x_int_2, y_int_1, y_int_2;
std::vector<std::array<double, 2>> intersection_points;
std::array<double, 2> last_point;
std::array<double, 2> second_last_point;
std::array<double, 2> target_point;

std::array<double, 2>
OdomChassis::findIntersectionPoint(std::vector<std::array<double, 2>> path,
                                   double radius) {
	intersection_points.clear();
	x1 = global_x;
	y1 = global_y;

	while (intersection_points.size() == 0) {
		for (int i = path.size() - 1; i >= 0; i--) {
			x2 = path[i - 1][0];
			y2 = path[i - 1][1];
			x3 = path[i][0];
			y3 = path[i][1];

			m = (y3 - y2) / (x3 - x2);
			// for non vertical line segments
			if (isfinite(m)) {
				a = x1 + m * m * x2 - m * y2 + m * y1;
				b = sqrt(-m * m * x1 * x1 + 2 * m * m * x1 * x2 + 2 * m * x1 * y1 -
				         2 * m * x1 * y2 + m * m * radius * radius + 2 * m * x2 * y2 +
				         radius * radius + 2 * y2 * y1 - m * m * x2 * x2 -
				         2 * m * x2 * y1 - y2 * y2 - y1 * y1);
				c = 1 + m * m;

				x_int_1 = (a - b) / c;
				x_int_2 = (a + b) / c;

				if (isfinite(x_int_1)) {
					if ((x2 < x3 && x_int_1 < x3 && x_int_1 > x2) ||
					    (x2 > x3 && x_int_1 > x3 && x_int_1 < x2)) {
						y_int_1 = m * (x_int_1 - x2) + y2;
						intersection_points.push_back({x_int_1, y_int_1});
					}
					if ((x2 < x3 && x_int_2 < x3 && x_int_2 > x2) ||
					    (x2 > x3 && x_int_2 > x3 && x_int_2 < x2)) {
						y_int_2 = m * (x_int_2 - x2) + y2;
						intersection_points.push_back({x_int_2, y_int_2});
					}
				}
			}
			// for vertical line segments
			else {
				a = sqrt(radius * radius - x2 * x2 + 2 * x2 * x1 - x1 * x1);
				y_int_1 = x1 - a;
				y_int_2 = x1 + a;
				x_int_1 = x2;

				if (isfinite(y_int_1)) {
					if ((x2 < x3 && x_int_1 < x3 && x_int_1 > x2) ||
					    (x2 > x3 && x_int_1 > x3 && x_int_1 < x2))
						intersection_points.push_back({x_int_1, y_int_1});
					if ((x2 < x3 && x_int_2 < x3 && x_int_2 > x2) ||
					    (x2 > x3 && x_int_2 > x3 && x_int_2 < x2))
						intersection_points.push_back({x_int_1, y_int_2});
				}
			}
			if (intersection_points.size() != 0) {
				last_segment = i == path.size() - 1;
				break;
			}
		}
		radius += 1; // 1 inch, might need to be adjusted
	}

	last_point = intersection_points.at(0);

	// if robot only intersects once on latest segment
	if (intersection_points.size() == 1) {
		target_point = last_point;
	}
	// if robot intersects twice on latest segment
	else {
		second_last_point = intersection_points.at(1);
		// if last segment is directed to the right
		if (x3 > x2) {
			if (last_point[0] > second_last_point[0])
				target_point = last_point;
			else
				target_point = second_last_point;
		}
		// if last segment is directed to the left
		else if (x2 > x3) {
			if (last_point[0] > second_last_point[0])
				target_point = second_last_point;
			else
				target_point = last_point;
		}
		// if last segment is vertical
		else {
			// if last segment is directed up
			if (y3 > y2) {
				if (last_point[1] > second_last_point[1])
					target_point = last_point;
				else
					target_point = second_last_point;
			}
			// if last segment is directed down
			else if (y2 > y3) {
				if (last_point[1] > second_last_point[1])
					target_point = second_last_point;
				else
					target_point = last_point;
			} else
				target_point = last_point;
		}
	}

	return target_point;
}

void OdomChassis::followPath(std::vector<std::array<double, 2>> path) {
	double max_speed = 80; // 100 max

	double inner_radius = 12.0;
	double outer_radius = 18.0;

	double kP_ang = 10.0;
	double kI_ang = 0.0;
	double kD_ang = 0.0;

	double kP_vel = 10.0;
	double kI_vel = 0.0;
	double kD_vel = 0.0;

	double ang_prev_error = 0;
	double vel_prev_error = 0;

	double left_prev = 0;
	double right_prev = 0;

	while (1) {
		std::array<double, 2> ang_tracking_point =
		    findIntersectionPoint(path, inner_radius);
		if (last_segment) {
			move(path[path.size() - 1]);
			break;
		}
		std::array<double, 2> vel_tracking_point =
		    findIntersectionPoint(path, outer_radius);

		double ang_error = getAngleError(ang_tracking_point);
		double vel_error = getAngleError(vel_tracking_point);

		double ang_derivative = ang_error - ang_prev_error;
		double vel_derivative = vel_error - vel_prev_error;

		ang_prev_error = ang_error;
		vel_prev_error = vel_error;

		double forward_speed;
		if (vel_error == 0) {
			forward_speed = max_speed;
		} else {
			forward_speed = kP_vel / vel_error - kD_vel * vel_derivative;
		}
		double turn_modifier = kP_ang * ang_error + kD_ang * ang_derivative;

		double left_speed = forward_speed + turn_modifier;
		double right_speed = forward_speed - turn_modifier;

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

		left_speed = slew(left_speed, accel_step, &left_prev);
		right_speed = slew(right_speed, accel_step, &right_prev);

		left_prev = left_speed;
		right_prev = right_speed;

		leftMotors->moveVoltage(left_speed * 120);
		rightMotors->moveVoltage(right_speed * 120);

		delay(10);
	}
}

} // namespace arms::odom

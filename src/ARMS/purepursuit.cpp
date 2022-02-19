#include "ARMS/api.h"
#include "api.h"

#include <cmath>
#include <limits>

using namespace pros;

// Resources Used:
//	* https://github.com/xiaoxiae/PurePursuitAlgorithm
//	*
//https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
//	* https://www.mathworks.com/help/robotics/ug/pure-pursuit-controller.html

namespace arms::purepursuit {

// Because floating point values rarely equal each other exactly,
// we use a small fudge range
const double EPSILON = 1e-3;

inline bool roughly_equal(double val, double target) {
	return (val > -EPSILON && val < EPSILON);
}

/**
 * 	@param orig The origin point of the ray to intersect with
 * 	@param dir	The direction that the ray travels in
 * 	@param cent	The center point of the circle
 * 	@param r	The radius of the circle
 *
 * 	@param t1	A reference to a double to hold the lower t parameter
 * 	@param t2	A reference to a double to hold the higher t parameter
 *
 * 	@return 	An integer representing how many intersections occured.
 * 				0, 1, or 2
 */
int rayCircleIntersectionTimes(Point orig, Point dir, Point cent, double r,
                               double& t1, double& t2) {
	Point omc = orig - cent;

	// The 'a' term of the quadratic is not needed becasue dir should be a unit
	// vector
	double b = 2 * dot(dir, omc);
	double c = dot(omc, omc) - r * r;

	double disc = b * b - 4.0 * c;

	if (roughly_equal(disc, 0.0))
		return 0;

	double rt = std::sqrt(disc);

	t1 = (-b + rt) * 0.5;
	t2 = (-b - rt) * 0.5;

	if (t1 > t2)
		std::swap(t1, t2);

	return disc < 0.0 ? 1 : 2;
}

int lineCircleIntersectionPoints(Point start, Point end, Point cent, double r,
                                 Point& p1, Point& p2) {
	double t1, t2;
	Point dir = normalize(end - start);
	int numInter = rayCircleIntersectionTimes(start, dir, cent, r, t1, t2);

	double len = length(dir);

	bool inside1 = (t1 > 0.0 && t1 < len);
	bool inside2 = (t2 > 0.0 && t2 < len);

	if (numInter == 2) {
		if (inside1 && inside2) {
			p1 = start + t1 * dir;
			p2 = start + t2 * dir;
			return 2;
		} else if (inside1) {
			p1 = start + t1 * dir;
			return 1;
		} else if (inside2) {
			p1 = start + t2 * dir;
			return 1;
		}
	} else if (numInter == 1 && inside1) {
		p1 = start + t1 * dir;
	}

	return 0;
}

Point getGoalPoint(std::vector<Point>& path, Point botPos, double dist) {
	Point closestPoint;
	double closestDist = std::numeric_limits<double>::max(); //max double value

	//loop through each of the line segments
	for(int i = 0; i < path.size() - 1; i++) {
		Point p1, p2;
		int numInter = lineCircleIntersectionPoints(path[i], path[i + 1], botPos, dist, p1, p2);

		if(numInter == 0) continue;

		double l1 = length2(p1 - botPos);

		if(l1 < closestDist) {
			closestPoint = p1;
			closestDist = l1;
		}
		
		double l2 = length2(p2 - botPos);
		if(numInter == 2 && l2 < closestDist) {
			closestPoint = p2;
			closestDist = l2;
		}
	}

	return closestPoint;
}

void followPath(std::vector<Point>&& path, double radius) {
	std::cout << "Following path:\n";

	for(Point& p : path)
		std::cout << "  (" << p.x << "," << p.y << "),\n";

	std::cout << std::endl;

	while(true) {
		Point botPos{odom::global_x, odom::global_y};
		Point target = getGoalPoint(path, botPos, radius);

		std::cout << "pos: (" << botPos.x << ", " << botPos.y << ")" << "\tgoal: (" <<
				  	 target.x << ", " << target.y << ")" << std::endl;

		chassis::move(target, arms::ASYNC);

		delay(10);
	}
}

} // namespace arms::purepursuit

#if 0

namespace arms::purepursuit {

bool last_segment = false;
double m, a, b, c;
double x1, x2, x3, y1, y2, y3;
double x_int_1, x_int_2, y_int_1, y_int_2;
std::vector<std::array<double, 2>> intersection_points;
std::array<double, 2> last_point;
std::array<double, 2> second_last_point;
std::array<double, 2> target_point;

bool lineCircleIntersection

std::array<double, 2>
findIntersectionPoint(std::vector<std::array<double, 2>> path, double radius) {
	intersection_points.clear();
	x1 = odom::global_x;
	y1 = odom::global_y;

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

void followPath(std::vector<std::array<double, 2>> path) {
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

	while (1) {
		std::array<double, 2> ang_tracking_point =
		    findIntersectionPoint(path, inner_radius);
		if (last_segment) {
			chassis::move(path[path.size() - 1], 80);
			break;
		}
		std::array<double, 2> vel_tracking_point =
		    findIntersectionPoint(path, outer_radius);

		double ang_error = odom::getAngleError(ang_tracking_point);
		double vel_error = odom::getAngleError(vel_tracking_point);

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

		left_speed =
		    chassis::slew(left_speed, chassis::slew_step, chassis::leftPrev);
		right_speed =
		    chassis::slew(right_speed, chassis::slew_step, chassis::rightPrev);

		chassis::leftMotors->moveVoltage(left_speed * 120);
		chassis::rightMotors->moveVoltage(right_speed * 120);

		delay(10);
	}
}

} // namespace arms::purepursuit
#endif
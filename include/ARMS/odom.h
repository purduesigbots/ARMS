#ifndef _ODOM_H_
#define _ODOM_H_

#include "ARMS/config.h"
#include <array>

namespace odom {

extern double global_x;
extern double global_y;
extern double heading;
extern double prev_right_pos;
extern double prev_left_pos;
extern double prev_middle_pos;

double getAngleError(std::array<double, 2> point);

double getDistanceError(std::array<double, 2> point);

void goToPointAsync(std::array<double, 2> point, double max = 80);

void goToPoint(std::array<double, 2> point, double max = 80);

void init(bool debug = DEBUG, double chassis_width = CHASSIS_WIDTH,
          double exit_error = EXIT_ERROR);

} // namespace odom

#endif

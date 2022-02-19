#ifndef _ARMS_ODOM_H_
#define _ARMS_ODOM_H_

#include "ARMS/point.h"
#include <array>

namespace arms::odom {

extern double global_x;
extern double global_y;
extern double heading;
extern double heading_degrees;
extern double prev_right_pos;
extern double prev_left_pos;
extern double prev_middle_pos;

void reset(Point point = {0, 0});

void reset(Point point, double angle);

double getAngleError(Point point);

double getDistanceError(Point point);

void init(bool debug, double left_right_distance, double middle_distance,
          double left_right_tpi, double middle_tpi);

} // namespace arms::odom

#endif

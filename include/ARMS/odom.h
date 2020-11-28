#ifndef _ODOM_H_
#define _ODOM_H_

#include "ARMS/config.h"
#include <array>

namespace odom {

extern double global_x;
extern double global_y;
extern double heading;
extern double heading_degrees;

double getAngle(std::array<double, 2> point);

double getDistance(std::array<double, 2> point);

void goToPoint(std::array<double, 2> point);

void init(bool debug = DEBUG, double chassis_width = CHASSIS_WIDTH);

} // namespace odom

#endif

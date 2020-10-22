#ifndef _ODOM_H_
#define _ODOM_H_

#include "greenhat/config.h"
#include <array>

namespace odom {

extern double global_x;
extern double global_y;
extern double heading;
extern double heading_degrees;

int odomTask();

double getAngle(std::array<double, 2> point);

double getDistance(std::array<double, 2> point);

void goToPoint(std::array<double, 2> point);

int slew(double speed);

} // namespace odom

#endif 
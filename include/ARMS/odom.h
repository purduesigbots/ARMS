#ifndef _ODOM_H_
#define _ODOM_H_

#include "ARMS/config.h"
#include <array>

namespace odom {

extern double global_x;
extern double global_y;
extern double heading;

double getAngleError(std::array<double, 2> point);

double getDistanceError(std::array<double, 2> point);

void goToPoint(std::array<double, 2> point);

void init(bool debug = DEBUG, double chassis_width = CHASSIS_WIDTH,
          double linearKP = GTP_LINEAR_KP, double linearKD = GTP_LINEAR_KD,
          double angularKP = GTP_ANGULAR_KP, double angularKD = GTP_ANGULAR_KD,
          double slew_step = GTP_SLEW_STEP, double exit_error = GTP_SLEW_STEP);

} // namespace odom

#endif

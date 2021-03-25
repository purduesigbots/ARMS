#ifndef _ODOM_H_
#define _ODOM_H_

#include "ARMS/config.h"
#include <array>

namespace odom {

extern double global_x;
extern double global_y;
extern double heading;
extern double heading_degrees;
extern double prev_right_pos;
extern double prev_left_pos;
extern double prev_middle_pos;

void reset(std::array<double, 2> point = {0, 0});

void reset(std::array<double, 2> point, double angle);

double getAngleError(std::array<double, 2> point);

double getDistanceError(std::array<double, 2> point);

void moveAsync(std::array<double, 2> point, double max = 80);

void holoAsync(std::array<double, 2> point, double angle, double max = 80,
               double turnMax = 50);

void move(std::array<double, 2> point, double max = 80);

void moveThru(std::array<double, 2> point, double max = 80);

void holo(std::array<double, 2> point, double angle, double max = 80,
          double turnMax = 50);

void holoThru(std::array<double, 2> point, double angle, double max = 80,
              double turnMax = 50);

void init(bool debug = ODOM_DEBUG,
          double left_right_distance = LEFT_RIGHT_DISTANCE,
          double middle_distance = MIDDLE_DISTANCE,
          double left_right_tpi = LEFT_RIGHT_TPI,
          double middle_tpi = MIDDLE_TPI, bool holonomic = HOLONOMIC,
          double exit_error = EXIT_ERROR);

} // namespace odom

#endif

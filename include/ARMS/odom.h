#ifndef _ARMS_ODOM_H_
#define _ARMS_ODOM_H_

#include <array>

namespace arms::odom {

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

void move(std::array<double, 2> point, double max = 80, bool settle = true);

void moveThru(std::array<double, 2> point, double max = 80);

void holo(std::array<double, 2> point, double angle, double max = 80,
          double turnMax = 50);

void holoThru(std::array<double, 2> point, double angle, double max = 80,
              double turnMax = 50);

void init(bool debug, double left_right_distance, double middle_distance,
          double left_right_tpi, double middle_tpi, bool holonomic,
          double exit_error);

} // namespace arms::odom

#endif

#ifndef _PURE_PURSUIT_H_
#define _PURE_PURSUIT_H_

#include <array>
#include <vector>

namespace purepursuit {

std::array<double,2> findIntersectionPoint(std::vector<std::array<double, 2>> path, double radius);

double getAngle(std::array<double, 2> point);
double getDistance(std::array<double, 2> point);

void goToPoint(std::vector<std::array<double, 2>> path);
void followPath(std::vector<std::array<double, 2>> path);

} // namespace purepursuit

#endif

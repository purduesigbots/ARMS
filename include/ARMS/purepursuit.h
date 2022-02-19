#ifndef _ARMS_PURE_PURSUIT_H_
#define _ARMS_PURE_PURSUIT_H_

#include "point.h"

#include <array>
#include <vector>
namespace arms::purepursuit {
    
void followPath(std::vector<Point>&& path, double radius);

}//namespace arms::purepursuit

#if 0
namespace arms::purepursuit {

std::array<double, 2>
findIntersectionPoint(std::vector<std::array<double, 2>> path, double radius);

void goToPoint(std::vector<std::array<double, 2>> path);
void followPath(std::vector<std::array<double, 2>> path);

} // namespace arms::purepursuit

#endif

#endif

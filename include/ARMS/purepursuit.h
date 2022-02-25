#ifndef _ARMS_PURE_PURSUIT_H_
#define _ARMS_PURE_PURSUIT_H_

#include "point.h"

#include <array>
#include <vector>
namespace arms::purepursuit {

Point getLookaheadPoint(std::vector<Point> waypoints);
void init(double lookahead);

} // namespace arms::purepursuit

#endif

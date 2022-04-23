#ifndef _ARMS_PURE_PURSUIT_H_
#define _ARMS_PURE_PURSUIT_H_

#include "point.h"

#include <array>
#include <vector>
namespace arms::purepursuit {

extern std::vector<Point> waypoints;
extern double lookahead;

Point getLookaheadPoint();
double getDistanceError();

void reset();

void init(double lookahead);

} // namespace arms::purepursuit

#endif

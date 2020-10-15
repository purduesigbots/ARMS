#ifndef _PURE_PURSUIT_H_
#define _PURE_PURSUIT_H_

#include "greenhat/config.h"
#include "okapi/api.hpp"

namespace purepursuit {

std::array<double,2> findIntersectionPoint(double path [][2], double radius);

} // namespace purepursuit

#endif

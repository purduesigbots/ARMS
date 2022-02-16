#ifndef _ARMS_POINT_ANGLE_H_
#define _ARMS_POINT_ANGLE_H_

#include "ARMS/api.h"

namespace arms {

/**
 * This is a helper union that defines a 2d point with an angle.
 * 
 * A union was chosen to represent the point so that a couple different names
 * can be used to access the same elements. For example:
 *      point.x, point.data[0], and point[0]
 * refer to the same variable.
 */
union PointAngle
{
    double& operator[](unsigned int index) {
        return data[index];
    }

    struct { double x, y, a; };
    Point p;
    double data[3];
};

}//namespace arms

#endif//_ARMS_POINT_ANGLE_H_
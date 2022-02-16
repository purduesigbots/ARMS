#ifndef _ARMS_POINT_H_
#define _ARMS_POINT_H_

#include <array>

namespace arms {

/**
 * This is a helper union that defines a 2d point or vector. It defines a
 * set of operators that can be used to perform vector operations more 
 * consisely.
 * 
 * A union was chosen to represent the point so that a couple different names
 * can be used to access the same elements. For example:
 *      point.x, point.data[0], and point[0]
 * refer to the same variable.
 */
union Point {
    Point operator-() {
        return {-x, -y};
    }

    Point operator+(Point& o) {
        return {x + o.x, y + o.y};
    }

    Point operator-(Point& o) {
        return {x - o.x, y - o.y};
    }

    Point operator*(Point& o) {
        return {x * o.x, y * o.y};
    }

    Point operator/(Point& o) {
        return {x / o.x, y / o.y};
    }

    Point& operator+=(Point& o) {
        x += o.x, y += o.y;
        return *this;
    }

    Point& operator-=(Point& o) {
        x -= o.x, y -= o.y;
        return *this;
    }
    
    Point& operator*=(Point& o) {
        x *= o.x, y *= o.y;
        return *this;
    }
    
    Point& operator/=(Point& o) {
        x /= o.x, y /= o.y;
        return *this;
    }

    double& operator[](unsigned int index) {
        return data[index];
    }

    //This is a stop gap until the codebase is made to use Point for bot
    //coordinates
    //
    // TODO: Replace PID code to use this class
    std::array<double, 2> std() { 
        return {x, y};
    }

    struct { double x, y; };
    double data[2];
};

inline Point operator*(double s, Point& v) {
    return {s * v.x, s * v.y};
}

inline Point operator*(Point& v, double s) {
    return {s * v.x, s * v.y};
}

inline Point operator/(Point& v, double s) {
    return {v.x / s, v.y / s};
}

inline Point& operator*=(Point& v, double s) {
    v.x *= s, v.y *= s;
    return v;
}

inline Point& operator/=(Point& v, double s) {
    v.x /= s, v.y /= s;
    return v;
}

inline double dot(Point& a, Point& b) {
    return a.x * b.x + a.y * b.y;
}

}//namespace arms

#endif//_ARMS_POINT_H_
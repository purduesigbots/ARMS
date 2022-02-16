#ifndef _ARMS_POINT_H_
#define _ARMS_POINT_H_

#include <array>

namespace arms {

/**
 * Represents a 2D point or vector.
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
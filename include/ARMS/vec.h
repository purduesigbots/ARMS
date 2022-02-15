#ifndef _ARMS_VEC_H_
#define _ARMS_VEC_H_

namespace arms {

/**
 * Represents a 2D point or vector.
 */
union Vec2 {

    Vec2 operator-() {
        return {-x, -y};
    }

    Vec2 operator+(Vec2& o) {
        return {x + o.x, y + o.y};
    }

    Vec2 operator-(Vec2& o) {
        return {x - o.x, y - o.y};
    }

    Vec2 operator*(Vec2& o) {
        return {x * o.x, y * o.y};
    }

    Vec2 operator/(Vec2& o) {
        return {x / o.x, y / o.y};
    }

    Vec2& operator+=(Vec2& o) {
        x += o.x, y += o.y;
        return *this;
    }

    Vec2& operator-=(Vec2& o) {
        x -= o.x, y -= o.y;
        return *this;
    }
    
    Vec2& operator*=(Vec2& o) {
        x *= o.x, y *= o.y;
        return *this;
    }
    
    Vec2& operator/=(Vec2& o) {
        x /= o.x, y /= o.y;
        return *this;
    }

    double& operator[](size_t index) {
        return data[index];
    }

    struct { double x, y; };
    double data[2];
};

inline Vec2 operator*(double s, Vec2& v) {
    return {s * v.x, s * v.y};
}

inline Vec2 operator*(Vec2& v, double s) {
    return {s * v.x, s * v.y};
}

inline Vec2 operator/(Vec2& v, double s) {
    return {v.x / s, v.y / s};
}

inline Vec2& operator*=(Vec2& v, double s) {
    v.x *= s, v.y *= s;
    return v;
}

inline Vec2& operator/=(Vec2& v, double s) {
    v.x /= s, v.y /= s;
    return v;
}

inline double dot(Vec2& a, Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

}//namespace arms

#endif//_ARMS_VEC_H_
#include "bezier.hpp"
#include "point.h"
Point Bezier::getPoint(double t) {
    double tt = t * t;
    double ttt = tt * t;

    Point point = points[0] * (-ttt + 3 * tt - 3 * t + 1) +
                  points[1] * (3 * ttt - 6 * tt + 3 * t) +
                  points[2] * (-3 * ttt + 3 * tt) +
                  points[3] * ttt;

    return point;
}

Point Bezier::getVelocity(double t) {
    double tt = t * t;

    Point point = points[0] * (-3 * tt + 6 * t - 3) +
                  points[1] * (9 * tt - 12 * t + 3) +
                  points[2] * (-9 * tt + 6 * t) +
                  points[3] * 3 * tt;

    return point;
}



double Bezier::getHeading(double t) {
    Point velocity = getVelocity(t);

    double heading = std::atan2(velocity.y, velocity.x);

    return heading;
}

double Bezier::getLength(double t) {
    double length = 0;

    for (double i = 0; i < t; i += 0.01) {
        Point velocity = getVelocity(i);
        length += std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y) * 0.01;
    }

    return length;
}
void Bezier::getPoses(){

    double dist_between_points = 1;
    double prev_point_dist = 0;
    double delta_t = 1000.0;
    for (int i = 0; i <=delta_t; i ++) {
        float t = i/delta_t;
        Point velocity = getVelocity(t);
        length += std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y) * delta_t;
        //std::cout << length << std::endl;
        if (length > prev_point_dist + dist_between_points){
            prev_point_dist+= dist_between_points;
            double heading = std::atan2(velocity.y, velocity.x);
            Point target = getPoint(t);
            Pose point = {target, heading};
            pose_list.push_back(point);
        }
        //std::cout << t<<std::endl;
        pose_list.back() = getPose(1);
    }

}
Pose Bezier::getPose(double t){
    Pose p = {getPoint(t),getHeading(t)};
    return p;
}
Point Bezier::getPointAtLength(double length) {
    double t = 0;

    for (double i = 0; i < length; i += 0.01) {
        Point velocity = getVelocity(t);
        i += sqrt(velocity.x * velocity.x + velocity.y * velocity.y) * 0.01;
        t += 0.01;
    }

    return getPoint(t);
}

double Bezier::getClosestPoint(Point p) {
    double t = 0;
    double minDistance = 1000000;

    for (double i = 0; i < 1; i += 0.01) {
        Point point = getPoint(i);
        double distance = sqrt((point.x - p.x) * (point.x - p.x) + (point.y - p.y) * (point.y - p.y));
        if (distance < minDistance) {
            minDistance = distance;
            t = i;
        }
    }

    return t;
}



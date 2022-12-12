#include "ARMS/api.h"
#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include "api.h"
#include <vector>

#define DELTA_D 1

namespace arms {

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
                  points[3] * (3 * tt);

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

void Bezier::getPoses() {
    double prev_point_dist = 0;
    int num_samples = 1000;
    double delta_t = 1.0 / num_samples;    
    length = 0;
    for (int i = 0; i <= num_samples; i ++) {
        float t = i * delta_t;
        Point velocity = getVelocity(t);
        length += std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y) * delta_t;

        if (length > prev_point_dist + DELTA_D){
            //std::cout << length << std::endl;
            prev_point_dist += DELTA_D;
            double heading = std::atan2(velocity.y, velocity.x);
            Point target = getPoint(t);
            Pose point = {target, heading};
            pose_list.push_back(point);
        }
        //std::cout << t << std::endl;
    }
    pose_list.back() = getPose(1);
}

Pose Bezier::getPose(double t) {
    Pose p = {getPoint(t),getHeading(t)};
    return p;
}

std::vector<Pose> Bezier::returnPoseList() {
    return pose_list;
}

int Bezier::get_point_num() {
    return pose_list.size();
}



void follow_bezier(Bezier path, double max_vel) {
    path.getPoses();
    std::vector<Pose> pose_list = path.returnPoseList();
    int size = path.get_point_num();
    double exit_error = 2; // in
    double target_vel =0; // in/sec
    double target_acc = ACCEL; // in/sec^2
    double prev_vel = 0;
    const float d2_req = DELTA_D;
    float max_speed = std::fmin(max_vel/100.0*MAX_VELOCITY, sqrt(2*ACCEL*DECEL*(std::abs(size*DELTA_D)-d2_req)/(ACCEL+DECEL))); //Limits max speed
    float dist_switch = (max_speed*max_speed-MIN_VELOCITY*MIN_VELOCITY)/(2*DECEL); // deceleration distance from end
    for (int i = 0; i < size; i++){
        target_vel = sqrt(DELTA_D * target_acc * 2.0 + prev_vel * prev_vel);
        if ((size-i-1) * DELTA_D <= dist_switch) {
            target_acc = -DECEL;
        }
        if (target_vel > max_speed) {
            target_vel = max_speed;
        }
        // chassis::move(pose_list[i].returnPose(),target_vel);
        // chassis::waitUntilFinished(exit_error);
        printf("vel: %f, acc: %f, i: %d,cond: %f\n", target_vel, target_acc, i, (size-i) * DELTA_D * -DECEL * 2.0);
        prev_vel = target_vel;
    }
}

}//namespace arms


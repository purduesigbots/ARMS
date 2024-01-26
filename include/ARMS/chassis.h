#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "../api.h"
#include "ARMS/flags.h"
#include "ARMS/point.h"
#include <memory>

namespace arms::chassis {

extern double maxSpeed;
extern double min_linear_speed;
extern double min_angular_speed;
extern std::shared_ptr<pros::Motor_Group> leftMotors;
extern std::shared_ptr<pros::Motor_Group> rightMotors;

/**
 * Set the brake mode for all chassis motors
 */
void setBrakeMode(pros::motor_brake_mode_e_t b);

/**
 * Return true if the chassis is not moving
 */
bool settled();

/**
 * Wait for the chassis to complete the current movement
 */
void waitUntilFinished(double exit_error);

/**
 * Perform 2D chassis movement
 */
void move(std::vector<double> target, double max, double exit_error, double lp,
          double ap, MoveFlags = NONE);
void move(std::vector<double> target, double max, double exit_error,
          MoveFlags = NONE);
void move(std::vector<double> target, double max, MoveFlags = NONE);
void move(std::vector<double> target, MoveFlags = NONE);

/**
 * Perform 1D chassis movement
 */
void move(double target, double max, double exit_error, double lp, double ap,
          MoveFlags = NONE);
void move(double target, double max, double exit_error, MoveFlags = NONE);
void move(double target, double max, MoveFlags = NONE);
void move(double target, MoveFlags = NONE);

/**
 * Perform a turn movement
 */
void turn(double target, double max, double exit_error, double ap,
          MoveFlags = NONE);
void turn(double target, double max, double exit_error, MoveFlags = NONE);
void turn(double target, double max, MoveFlags = NONE);
void turn(double target, MoveFlags = NONE);

/**
 * Turn to face a point
 */
void turn(Point target, double max, double exit_error, double ap,
          MoveFlags = NONE);
void turn(Point target, double max, double exit_error, MoveFlags = NONE);
void turn(Point target, double max, MoveFlags = NONE);
void turn(Point target, MoveFlags = NONE);

void moveVectorEnd(double magnitude, double angle, double max, double exit_error,
                   double lp, double ap, MoveFlags = NONE);
void moveVectorEnd(double magnitude, double angle, double max, double exit_error,
                   MoveFlags = NONE);
void moveVectorEnd(double magnitude, double angle, double max, MoveFlags = NONE);
void moveVectorEnd(double magnitude, double angle, MoveFlags = NONE);

void moveVectorPath(double magnitude, double angle, double max, double exit_error,
                    double lp, double ap, MoveFlags = NONE);
void moveVectorPath(double magnitude, double angle, double max, double exit_error,
                    MoveFlags = NONE);
void moveVectorPath(double magnitude, double angle, double max, MoveFlags = NONE);
void moveVectorPath(double magnitude, double angle, MoveFlags = NONE);



/**
 * Assign a power to the left and right motors
 */
void tank(double left, double right, bool velocity = false);

/**
 * Assign a vertical and horizontal power to the motors
 */
void arcade(double vertical, double horizontal, bool velocity = false);

/**
 * initialize the chassis
 */
void init(std::initializer_list<int8_t> leftMotors,
          std::initializer_list<int8_t> rightMotors,
          pros::motor_gearset_e_t gearset, double slew_step,
          double linear_exit_error, double angular_exit_error,
          double settle_thresh_linear, double settle_thresh_angular,
          int settle_time, double min_linear_speed, double min_angular_speed);

} // namespace arms::chassis

#endif

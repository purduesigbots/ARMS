#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "ARMS/flags.h"
#include "ARMS/point.h"
#include "okapi/api.hpp"

/*!

    * @brief The chassis subsystem.
    *
    * @details This namespace stores all functions and motors used
    * to control the chassis.
    */
namespace arms::chassis {

extern double maxSpeed;
extern double leftPrev;
extern double rightPrev;
extern double slew_step;
extern std::shared_ptr<okapi::MotorGroup> leftMotors;
extern std::shared_ptr<okapi::MotorGroup> rightMotors;

/**
 * Set the speed of target motor
 */
void motorMove(std::shared_ptr<okapi::MotorGroup> motor, double speed,
               bool vel);

/**
 * Set the brake mode for all chassis motors
 */
void setBrakeMode(okapi::AbstractMotor::brakeMode b);

/**
 * Reset imu if it is being used
 */
void resetAngle(double angle = 0);

/**
 * Reduce an input speed if it exceeds the max value
 */
double limitSpeed(double speed, double max);

/**
 * Get a gradually accelerating speed towards the target input
 */
double slew(double speed, double step, double prev);

/**
 * Return true of the chassis is not moving
 */
bool settled();

/**
 * Wait for the chassis to complete the current movement
 */
void waitUntilFinished(double exit_error);

/**
 * Perform a chassis movement
 */
void move(Point target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);
void move(Point target, double max, double exit_error,
          MoveFlags = NONE);
void move(Point target, double max, MoveFlags = NONE);
void move(Point target, MoveFlags = NONE);

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
void init(std::initializer_list<okapi::Motor> leftMotors,
          std::initializer_list<okapi::Motor> rightMotors, int gearset,
          double slew_step, double linear_exit_error, double angular_exit_error, 
          double settle_thresh_linear, double settle_thresh_angular,
          int settle_time);

} // namespace arms::chassis

#endif

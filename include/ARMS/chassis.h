#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "ARMS/flags.h"
#include "ARMS/point.h"
#include "okapi/api.hpp"

namespace arms::chassis {

enum EncoderType { ENCODER_ADI, ENCODER_ROTATION };

extern double maxSpeed;
extern double leftPrev;
extern double rightPrev;
extern double slew_step;

// motors
extern std::shared_ptr<okapi::MotorGroup> leftMotors;
extern std::shared_ptr<okapi::MotorGroup> rightMotors;

// sensors
extern std::shared_ptr<okapi::ContinuousRotarySensor> leftEncoder;
extern std::shared_ptr<okapi::ContinuousRotarySensor> rightEncoder;
extern std::shared_ptr<okapi::ContinuousRotarySensor> middleEncoder;
extern std::shared_ptr<pros::Imu> imu;

/**
 *  Functions to interact with the non-motor encoders on the chassis.
 *  These should be used instead of accessing the encoders directly, as
 *  the chassis has the ability to use either ADI or Rotation encoders
 */

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
 * Reset the internal motor encoders for all chassis motors
 */
void reset();

/**
 * Return the raw encoder values
 */
std::array<double, 2> getEncoders();

/**
 * Return the distance traveled by the chassis
 */
double distance();

/**
 * Get the angle of the chassis in degrees
 */
double angle();

/**
 * Reduce an input speed if it exceeds the max value
 */
double limitSpeed(double speed, double max);

/**
 * Get a gradually accelerating speed towards the target input
 */
double slew(double speed, double step, double prev);

/**
 * Wait for the chassis to complete the current movement
 */
void waitUntilFinished(double exit_error);

/**
 * Perform a chassis movement
 */
void move(std::vector<Point> waypoints, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);
void move(std::vector<Point> waypoints, double max, double exit_error,
          MoveFlags = NONE);
void move(std::vector<Point> waypoints, double max, MoveFlags = NONE);
void move(std::vector<Point> waypoints, MoveFlags = NONE);

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
          double distance_constant, double degree_constant, double slew_step,
          int imuPort, std::tuple<int, int, int> encoderPorts, int expanderPort,
          double exit_error, int encoderType);

} // namespace arms::chassis

#endif

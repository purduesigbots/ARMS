#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "okapi/api.hpp"

#include "ARMS/vec.h"

namespace arms::chassis {

enum MoveFlags {
    ASYNC       = 0b0000'0001,
    THRU        = 0b0000'0010,
    BACKWARD    = 0b0000'0100,
    ABSOLUTE    = 0b0000'1000,
};

typedef uint32_t flags_t;

extern double maxSpeed;
extern double leftPrev;
extern double rightPrev;
extern double slew_step;

// motors
extern std::shared_ptr<okapi::MotorGroup> leftMotors;
extern std::shared_ptr<okapi::MotorGroup> rightMotors;

// sensors
extern std::shared_ptr<pros::ADIEncoder> leftEncoder;
extern std::shared_ptr<pros::ADIEncoder> rightEncoder;
extern std::shared_ptr<pros::ADIEncoder> middleEncoder;
extern std::shared_ptr<pros::Imu> imu;

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
 * Reduce a speed
 */
double limitSpeed(double speed, double max);

/**
 * Get a gradually accelerating speed towards the target input
 */
double slew(double speed, double step, double prev);

/**
 * Wait for the chassis to complete the current movement
 */
void waitUntilFinished(double exit_error = 0);

/**
 * Perform a linear chassis movement
 */
void move(double target, double max, double exitError, double kp, flags_t flags);
void move(double target, double max, double exitError, flags_t flags);
void move(double target, double max, flags_t flags);
void move(double target, flags_t flags);

/**
 * Perform an odom chassis movement
 */
void move(Vec2 target, double max, double exit_error, double lp, double ap, 
		  flags_t flags);
void move(Vec2 target, double max, double exit_error, flags_t flags);
void move(Vec2 target, double max, flags_t flags);
void move(Vec2 target, flags_t flags);

/**
 * Perform a turn movement
 */
void turn(double target, int max, double exit_error, double ap, flags_t flags);
void turn(double target, int max, double exit_error, flags_t flags);
void turn(double target, int max, flags_t flags);
void turn(double target, flags_t flags);

/**
 * Turn to face a point
 */
void turn(Vec2 target, int max, double exit_error, double ap, flags_t flags);
void turn(Vec2 target, int max, double exit_error, flags_t flags);
void turn(Vec2 target, int max, flags_t flags);
void turn(Vec2 target, flags_t flags);

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
          double arc_slew_step, int imuPort,
          std::tuple<int, int, int> encoderPorts, int expanderPort,
          double exit_error);

} // namespace arms::chassis

#endif

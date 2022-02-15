#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "okapi/api.hpp"

namespace arms::chassis {

extern double maxSpeed;
extern double leftPrev;
extern double rightPrev;

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
void motorMove(std::shared_ptr<okapi::Motor> motor, double speed, bool vel);

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
double slew(double speed, double step, double* prev);

/**
 * Perform a linear chassis movement
 */
void move(double sp, double max, std::array<double, 2> pid, double exit_error,
          bool thru, bool blocking);

/**
 * Perform an odom chassis movement
 */
void move(std::array<double, 2> sp, double max,
          std::array<double, 2> linear_pid, std::array<double, 2> angular_pid,
          double exit_error, bool thru, bool direction, bool blocking);

/**
 * Perform a turn movement
 */
void turn(double sp, int max = 100);

/**
 * Perform an absolute turn movement
 */
void turnAbsolute(double sp, int max = 100);

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
          double distance_constant, double degree_constant, int settle_time,
          double settle_threshold_linear, double settle_threshold_angular,
          double slew_step, double arc_slew_step, int imuPort,
          std::tuple<int, int, int> encoderPorts, int expanderPort);

} // namespace arms::chassis

#endif

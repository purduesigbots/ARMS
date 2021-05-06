#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "ARMS/config.h"
#include "okapi/api.hpp"

namespace chassis {

extern bool useVelocity;
extern double accel_step;
extern double distance_constant;
extern double width;
extern double maxSpeed;
extern double maxTurn;

extern std::shared_ptr<okapi::Motor> frontLeft;
extern std::shared_ptr<okapi::Motor> frontRight;
extern std::shared_ptr<okapi::Motor> backLeft;
extern std::shared_ptr<okapi::Motor> backRight;

extern std::shared_ptr<okapi::MotorGroup> leftMotors;
extern std::shared_ptr<okapi::MotorGroup> rightMotors;

extern std::shared_ptr<pros::ADIEncoder> leftEncoder;
extern std::shared_ptr<pros::ADIEncoder> rightEncoder;
extern std::shared_ptr<pros::ADIEncoder> middleEncoder;
extern std::shared_ptr<pros::Imu> imu;

/**
 * Set the speed of target motor
 */
void motorMove(std::shared_ptr<okapi::Motor> motor, double speed, bool vel);

/**
 * Set the speed of target motor group
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
 * Get the average position between the sides of the chassis
 */
double position(bool yDirection = false);

/**
 * Get the angle of the chassis in degrees
 */
double angle();

/**
 * Get the difference between the sides of the chassis
 */
double difference();

/**
 * Reduce a speed
 */
double limitSpeed(double speed, double max);

/**
 * Get a gradually accelerating speed towards the target input
 */
double slew(double speed, double step, double* prev);

/**
 * Get a boolean that is true if the chassis motors are in motion
 */
bool settled();

/**
 * Delay the program until the chassis motors come to rest
 */
void waitUntilSettled();

/**
 * Begin an asycronous chassis movement
 */
void moveAsync(double sp, int max = 100);

/**
 * Begin an asycronous turn movement
 */
void turnAsync(double sp, int max = 100);

/**
 * Begin an asycronous absolute turn movement (only works with IMU)
 */
void turnAbsoluteAsync(double sp, int max = 100);

/**
 * Begin an asycronous holonomic chassis movement
 */
void holoAsync(double distance, double angle, int max = 100);

/**
 * Perform a chassis movement and wait until settled
 */
void move(double sp, int max = 100);

/**
 * Perform a turn movement and wait until settled
 */
void turn(double sp, int max = 100);

/**
 * Perform an absolute turn movement and wait until settled (only works with
 * IMU)
 */
void turnAbsolute(double sp, int max = 100);

/**
 * Perform a holonomic movement and wait until settled
 */
void holo(double distance, double angle, int max = 100);

/**
 * Move a distance at a set voltage with no PID
 */
void fast(double sp, int max = 100);

/**
 * Move for a duration at a set voltage with no PID
 */
void voltage(int t, int left_speed = 100, int right_speed = 101);

/**
 * Move for a duration at a set velocity using internal PID
 */
void velocity(int t, int left_max = 100, int right_max = 101);

/**
 * Assign a voltage to each motor on a scale of -100 to 100
 */
void tank(double left, double right);

/**
 * Assign a voltage to each motor on a scale of -100 to 100
 */
void arcade(double vertical, double horizontal);

/**
 * Assign a voltage to each motor on a scale of -100 to 100
 */
void holonomic(double x, double y, double z);

/**
 * initialize the chassis
 */
void init(std::initializer_list<okapi::Motor> leftMotors = {LEFT_MOTORS},
          std::initializer_list<okapi::Motor> rightMotors = {RIGHT_MOTORS},
          int gearset = GEARSET, double distance_constant = DISTANCE_CONSTANT,
          double degree_constant = DEGREE_CONSTANT,
          int settle_time = SETTLE_TIME,
          double settle_threshold_linear = SETTLE_THRESHOLD_LINEAR,
          double settle_threshold_angular = SETTLE_THRESHOLD_ANGULAR,
          double accel_step = ACCEL_STEP, double arc_step = ARC_STEP,
          int imuPort = IMU_PORT,
          std::tuple<int, int, int> encoderPorts = {ENCODER_PORTS},
          int expanderPort = EXPANDER_PORT,
          int joystick_threshold = JOYSTICK_THRESHOLD);

} // namespace chassis

#endif

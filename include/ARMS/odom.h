#ifndef _ARMS_ODOM_H_
#define _ARMS_ODOM_H_

#include "ARMS/point.h"

namespace arms::odom {

enum EncoderType { ENCODER_ADI, ENCODER_ROTATION };

// sensors
extern std::shared_ptr<okapi::ContinuousRotarySensor> leftEncoder;
extern std::shared_ptr<okapi::ContinuousRotarySensor> rightEncoder;
extern std::shared_ptr<okapi::ContinuousRotarySensor> middleEncoder;
extern std::shared_ptr<pros::Imu> imu;

extern bool reverse;

/**
 * Return the robot position coordinates
 */
Point getPosition();

/**
 * Return the robot heading
 */
double getHeading(bool radians = false);

/**
 * Reset the robot position to a desired coordinate
 */
void reset(Point point = {0, 0});

/**
 * Reset the robot position and heading to desired values
 */
void reset(Point point, double angle);

/**
 * Return the angle between the robots current heading and a point
 */
double getAngleError(Point point);

/**
 * Return the distance between the robot and a point
 */
double getDistanceError(Point point);

/**
 * Initialize the odometry
 */
void init(bool debug, int encoderType, std::array<int, 3> encoderPorts,
          int expanderPort, int imuPort, double left_right_distance,
          double middle_distance, double left_right_tpi, double middle_tpi);

} // namespace arms::odom

#endif

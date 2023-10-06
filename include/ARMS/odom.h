#ifndef _ARMS_ODOM_H_
#define _ARMS_ODOM_H_

#include "ARMS/point.h"
#include <memory>

namespace arms::odom {

typedef enum EncoderType { ENCODER_ADI, ENCODER_ROTATION } EncoderType_e_t;

// Odom Configuration
typedef struct config_data_s {
	int expanderPort = 0;
    int rightEncoderPort = 0;
    int leftEncoderPort = 0;
	int middleEncoderPort = 0;
    int imuPort = 0;
    EncoderType_e_t encoderType;
} config_data_s_t;

// sensors
extern std::shared_ptr<pros::Imu> imu;

/**
 * Return the left encoder position
 */
double getLeftEncoder();

/**
 * Return the right encoder position
 */
double getRightEncoder();

/**
 * Return the middle encoder position
 */
double getMiddleEncoder();

/**
 * Return the robot position coordinates
 */
Point getPosition();

/**
 * Return the desired robot position coordinates
*/
Point getDesiredPosition();


/**
 * Set the desired robot position coordinates
*/
void setDesiredPosition(Point point);

/**
 * Set the desired robot heading
 */
void setDesiredHeading(double angle);

/**
 * Return the robot heading
 */
double getHeading(bool radians = false);

/**
 * Return the desired robot heading
 */
double getDesiredHeading(bool radians = false);

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
void init(bool debug, EncoderType_e_t encoderType, std::array<int, 3> encoderPorts,
          int expanderPort, int imuPort, double track_width,
          double middle_distance, double tpi, double middle_tpi);

} // namespace arms::odom

#endif

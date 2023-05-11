#ifndef _ARMS_ODOM_H_
#define _ARMS_ODOM_H_

#include "ARMS/point.h"
#include "pros/imu.hpp"
#include <memory>

/*!
    * @namespace arms::odom
    *
    * @brief This namespace deals with the odometry system.
    * @details 
    * It is used to track the robot's position on the field.
    * 
    * 
*/
namespace arms::odom {

/*!
    * @typedef enum EncoderType_e_t
    * 
    * @details This enum is used to specify the type of encoder used. The options are:
    * ENCODER_ADI or ENCODER_ROTATION
    * 
    * This is used for setup in \ref arms::config::ENCODER_TYPE
*/
typedef enum EncoderType { ENCODER_ADI, ENCODER_ROTATION } EncoderType_e_t;

// Odom Configuration
/*!
    * @typedef struct config_data_s
    *
    * @details This struct is used to store the configuration data for the odometry system.
    * 
    * @var config_data_s_t::expanderPort
    * The port that the expander is plugged into
    * 
    * @var config_data_s_t::rightEncoderPort
    * The port that the right encoder is plugged into
    * 
    * @var config_data_s_t::leftEncoderPort
    * The port that the left encoder is plugged into
    * 
    * @var config_data_s_t::middleEncoderPort
    * The port that the middle encoder is plugged into
    * 
    * @var config_data_s_t::imuPort
    * The port that the IMU is plugged into
    * 
    * @var config_data_s_t::encoderType
    * The type of encoder used. This is used to determine how to read the encoder values
*/
typedef struct config_data_s {
	int expanderPort = 0;
    int rightEncoderPort = 0;
    int leftEncoderPort = 0;
	int middleEncoderPort = 0;
    int imuPort = 0;
    EncoderType_e_t encoderType;
} config_data_s_t;

// sensors
/*!
    * @var std::shared_ptr<pros::Imu> imu
    * The IMU sensor. This is used by odom to get the robot's heading.
*/
extern std::shared_ptr<pros::Imu> imu;

/*!
    * @fn double getLeftEncoder()
    * 
    * @details Return the left encoder position
*/
double getLeftEncoder();

/*!
    * @fn double getRightEncoder()
    * 
    * @details Return the right encoder position
*/
double getRightEncoder();

/*!
    * @fn double getMiddleEncoder()
    * 
    * @details Return the middle encoder position
*/
double getMiddleEncoder();

/*!
    * @fn Point getPosition()
    * 
    * @details Return a \ref Point representing the robot's position on the field
*/
Point getPosition();

/*!
    * @fn double getHeading(bool radians = false)
    *
    * @param radians If true, the heading will be returned in radians. If false, the heading will be returned in degrees.
    * 
    * @details Return the robot's heading based in degrees or radians, depending on the \a radians option
*/
double getHeading(bool radians = false);

/*!
    * @fn void reset(Point point = {0, 0})
    * 
    * @param point The position to reset the robot to
    * 
    * <b>Example:</b>
    * @code
    * // reset the robot's position to be at x: 24 inches, y: 0 inches 
    * arms::odom::reset({24, 0});
    * @endcode
    * 
    * @details Reset the odometry system to the provided position.
*/
void reset(Point point = {0, 0});

/*!
    * @fn void reset(Point point, double angle)
    *
    * @param point The position to reset the robot to
    * 
    * @param angle The angle to reset the robot to
    * 
    * <b>Example:</b>
    * @code
    * // reset the robot's position to be at x: 24 inches, y: 0 inches, and angle: 90 degrees
    * arms::odom::reset({24, 0}, 90);
    * @endcode
    * 
    * @details Reset the odometry system to the provided position and angle.
*/
void reset(Point point, double angle);

/*!
    * @fn double getAngleError(Point point)
    *
    * @param point The point we want to get the angle error of
    * 
    * <b>Example:</b>
    * @code
    * // get the angle error of the point (24, 0)
    * double error = arms::odom::getAngleError({24, 0});
    * @endcode
    * 
    * @details Return the angle error between the robot's current heading and the angle to the provided point.
*/
double getAngleError(Point point);

/*!
    * @fn double getDistanceError(Point point)
    *
    * @param point The point we want to get the distance error of
    * 
    * <b>Example:</b>
    * @code
    * // get the distance error of the point (24, 0)
    * double error = arms::odom::getDistanceError({24, 0});
    * @endcode
    *   
    * @details Return the distance error between the robot's current position and the provided point.
*/
double getDistanceError(Point point);

/// @cond DO_NOT_DOCUMENT
void init(bool debug, EncoderType_e_t encoderType, std::array<int, 3> encoderPorts,
          int expanderPort, int imuPort, double track_width,
          double middle_distance, double tpi, double middle_tpi);
/// @endcond

} // namespace arms::odom

#endif

#ifndef _ARMS_CONFIG_H_
#define _ARMS_CONFIG_H_

/// @cond DO_NOT_DOCUMENT
#include "ARMS/lib.h"
#include "okapi/api.hpp"
/// @endcond

namespace arms {
/*!
 * @file config.h
 * @brief <B>The ARMS configuration file. This file is where you setup everything about your chassis,
 * including the motors, the sensors, and constants.
 * This is also where you will setup autonomous selector.</B>
 *
 */

// Debug
/*!
 * @brief Odom debug mode
 *
 * @param db Enable/disable odom debug
 * 
 * <b>Example 1:</b>
 * @code
 * //enable odom debug messages
 * #define ODOM_DEBUG 1
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //disable odom debug messages
 * #define ODOM_DEBUG 0
 * @endcode
 *
 * @details Enable/disable odometry debugging messages being sent to the terminal. This can be useful when trying to troubleshoot chassis movements.
 */
#define ODOM_DEBUG db

// Negative numbers mean reversed motor
/*!
 * @brief Left chassis motors
 *
 * @param ports the motor ports on the right side of the chassis
 * 
 * <b> Example 1: </b>
 * @code
 * // two motors on the left side of the chassis in ports 4 and 5
 * #define LEFT_MOTORS 4, 5
 * @endcode
 * 
 * <b> Example 2: </b>
 * @code
 * // three motors on the left side of the chassis in ports 4, 5, and 6. the motor in port 5 is reversed
 * #define LEFT_MOTORS 4, -5, 6
 * @endcode
 * 
 * @details Comma seperated ports that the chassis's left motors are in. Negative values reverse the motor on that port.
 */
#define LEFT_MOTORS ports

/*!
 * @brief Right chassis motors
 *
 * @param ports the motor ports on the right side of the chassis
 * 
 * <b> Example 1: </b>
 * @code
 * // two motors on the right side of the chassis in ports 1 and 2
 * #define RIGHT_MOTORS 1, 2
 * @endcode
 * 
 * <b> Example 2: </b>
 * @code
 * // three motors on the right side of the chassis in ports 1, 2, and 3. the motor in port 2 is reversed
 * #define RIGHT_MOTORS 1, -2, 3
 * @endcode
 * 
 * @details Comma seperated ports that the chassis's right motors are in. Negative values reverse the motor on that port.
 */
#define RIGHT_MOTORS ports

/*!
 * @brief Chassis gearset
 *
 * @param rpm the rpm of the chassis's motors
 * 
 * <b>Example 1:</b>
 * @code
 * //using 200 rpm cartridges
 * #define GEARSET 200
 * @endcode
 *
 * @details Sets the robot's chassis gearset to \a rpm.
 */
#define GEARSET rpm // RPM of chassis motors

/*!
 * @brief Distance Constant
 *
 * @param dist the robot's distance constant
 * 
 * <b>Example 1:</b>
 * @code
 * //use 60 encoder ticks per unit
 * #define DISTANCE_CONSTANT 60
 * @endcode
 *
 * @details Sets the robot's distance constant to \a dist.
 */
#define DISTANCE_CONSTANT dist // Ticks per distance unit

// Unit constants
/*!
 * @brief Degree Constant
 *
 * @param deg the robot's degree constant 
 * 
 * <b>Example 1:</b>
 * @code
 * //using an IMU for robot's heading
 * #define DEGREE_CONSTANT 1
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //use 15 encoder ticks per degree
 * #define DEGREE_CONSTANT 15
 * @endcode
 *
 * @details Sets the robot's degree constant to \a deg.
 */
#define DEGREE_CONSTANT deg   // Ticks per degree (should be 1 if using an IMU)

// Sensors
/*!
 * @brief IMU Port
 *
 * @param p the port the imu is plugged into. Set to 0 for disabled
 * 
 * <b>Example 1:</b>
 * @code
 * //using an IMU in port 8
 * #define IMU_PORT 8
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //don't use an IMU 
 * #define IMU_PORT 0
 * @endcode
 *
 * @details Sets the chassis' imu to the sensor in port \a p.
 */
#define IMU_PORT 0                           // Port 0 for disabled

/*!
 *
 * @brief Encoder Ports
 *
 * @param left the port the left encoder is plugged into. 0 for disabled.
 * 
 * @param middle the port the middle encoder is plugged into. 0 for disabled.
 *
 * @param right the port the right encoder is plugged into. 0 for disabled.
 *  
 * <b>Example 1:</b>
 * @code
 * //using one parrallel encoder. IMU for heading and not worried about sidways movement
 * #define ENCODER_PORTS 1
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //don't use an IMU 
 * #define IMU_PORT 0
 * @endcode
 *
 * @details Sets up the 1-3 encoders being used on the bot. At least 1 encoder parrallel to the chassis must be used for odometry to work.
 * If either left or right encoder is blank, an IMU must also be used. An encoder perpindicular to the chassis (middle) should be used if the robot
 * is expected to be pushed sideways. Negative values reverse the direction of the encoder. The values should be valid smart ports if using the V5 
 * rotation sensor, or odd numbered adi ports if using the optical shaft encoders. This is configured at \ref ENCODER_TYPE
 */

#define ENCODER_PORTS left, middle, right    // Port 0 for disabled,
#define EXPANDER_PORT 0                      // Port 0 for disabled

/*!
 * @brief Encoder Type
 *
 * @param type The type of encoder being used
 *
 * <b>Example 1:</b>
 * @code
 * //using the new V5 Rotation sensors
 * #define ENCODER_TYPE arms::odom::ENCODER_ROTATION
 * @endcode
 *
 * <b>Example 2:</b>
 * @code
 * //using the old Optical Shaft Encoders
 * #define ENCODER_TYPE arms::odom::ENCODER_ADI
 * @endcode
 * @details Which type of vex encoder is being used on the chassis. Using a mixture of encoder types is not currently supported.
 * This influences what the valid values for \ref ENCODER_PORTS are.
 */
#define ENCODER_TYPE arms::odom::ENCODER_ADI // The type of encoders

// Odometry
#define LEFT_RIGHT_DISTANCE 0 // Distance between left and right tracking wheels
#define MIDDLE_DISTANCE 0     // Distance from middle wheel to turning center
#define MIDDLE_TPI 1          // Ticks per inch of middle wheel

// Movement tuning
#define SLEW_STEP 8          // Smaller number = more slew
#define LINEAR_EXIT_ERROR 1  // default exit distance for linear movements
#define ANGULAR_EXIT_ERROR 1 // default exit distance for angular movements
#define SETTLE_THRESH_LINEAR .5      // amount of linear movement for settling
#define SETTLE_THRESH_ANGULAR 1      // amount of angular movement for settling
#define SETTLE_TIME 250      // amount of time to count as settled
#define LINEAR_KP 1
#define LINEAR_KI 0
#define LINEAR_KD 0
#define TRACKING_KP 60		 // point tracking turning strength
#define ANGULAR_KP 1
#define ANGULAR_KI 0
#define ANGULAR_KD 0
#define MIN_ERROR 5          // Minimum distance to target before angular componenet is disabled

// Auton selector configuration constants
#define AUTONS "Front", "Back", "Do Nothing" // Names of autonomi, up to 10
#define HUE 0     // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton selected

// Initializer
inline void init() {

	chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, DISTANCE_CONSTANT,
	              DEGREE_CONSTANT, SLEW_STEP, LINEAR_EXIT_ERROR,
	              ANGULAR_EXIT_ERROR, SETTLE_THRESH_LINEAR, SETTLE_THRESH_ANGULAR, SETTLE_TIME);

	odom::init(ODOM_DEBUG, ENCODER_TYPE, {ENCODER_PORTS}, EXPANDER_PORT, IMU_PORT,
	           LEFT_RIGHT_DISTANCE, MIDDLE_DISTANCE, DISTANCE_CONSTANT,
	           MIDDLE_TPI);

	pid::init(LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, TRACKING_KP, MIN_ERROR);

	const char* b[] = {AUTONS, ""};
	selector::init(HUE, DEFAULT, b);

}

} // namespace arms

#endif

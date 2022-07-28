#ifndef _ARMS_CONFIG_H_
#define _ARMS_CONFIG_H_

/// @cond DO_NOT_DOCUMENT
#include "ARMS/lib.h"
#include "okapi/api.hpp"
#include "pid.h"
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
 * // three motors on the left side of the chassis in ports 4, 5, and 6. The motor in port 5 is reversed
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
 * // three motors on the right side of the chassis in ports 1, 2, and 3. The motor in port 2 is reversed
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
 * @param port the port the imu is plugged into. Set to 0 for disabled
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
 * @details Sets the chassis' imu to the sensor in port \a port.
 */
#define IMU_PORT port                           // Port 0 for disabled

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
 * #define ENCODER_PORTS 1, 0, 0
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //using all three encoders. No IMU for heading, so we will need to get our heading through encoders
 * #define ENCODER_PORTS 1, 3, 5
 * @endcode
 *
 * @details Sets up the 1-3 encoders being used on the bot. At least 1 encoder parrallel to the chassis must be used for odometry to work.
 * If either left or right encoder is set to 0, an IMU must also be used. An encoder perpindicular to the chassis (middle) should be used if the robot
 * is expected to be pushed sideways. Negative values reverse the direction of the encoder. A value of 0 disables the encoder. The values should be 
 * valid smart ports if using the V5 rotation sensor, or odd numbered adi ports if using the optical shaft encoders. This is configured at \ref ENCODER_TYPE
 * If all encoders are disabled, the integrated encoders in the chassis motors will be used.
 */
#define ENCODER_PORTS left, middle, right    // Port 0 for disabled

/*!
 *
 * @brief Encoder ADI Expander Port
 *
 * @param port the port the ADI expander is plugged into. 0 for disabled.
 *
 * <b>Example 1:</b>
 * @code
 * //using an ADI expander in port 1
 * #define ADI_PORT 1
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //don't use an ADI expander
 * #define ADI_PORT 0
 * @endcode
 *
 * @details Uses the expander port in port \a port for the encoder's configured at \ref ENCODER_PORTS.
 */
#define EXPANDER_PORT port                     // Port 0 for disabled

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
#define ENCODER_TYPE type // The type of encoders

// Odometry

/*!
 *
 * @brief Left Right Distance
 *
 * @param dist the distance between the left and right tracking wheels
 *
 * <b>Example 1:</b>
 * @code
 * //using a distance of 10 inches between the left and right tracking wheels
 * #define LEFT_RIGHT_DISTANCE 10
 * @endcode
 *
 * @details Sets the distance between the left and right tracking wheels to \a dist. Should be set to 0 if only using 1 tracker wheel
 */
#define LEFT_RIGHT_DISTANCE dist // Distance between left and right tracking wheels

/*!
 *
 * @brief Middle Distance
 *
 * @param dist the distance between the middle tracking wheel and the turning center of the chassis
 *
 * <b>Example 1:</b>
 * @code
 * //using a distance of 7 inches between the middle tracking wheel and the turning center of the chassis
 * #define MIDDLE_DISTANCE 7
 * @endcode
 *
 * @details Sets the distance between the middle tracking wheel and the turning center of the chassis to \a dist. Should be set to 0 if not using a middle tracker wheel.
 */
#define MIDDLE_DISTANCE dist     // Distance from middle wheel to turning center

/*!
 *
 * @brief Middle TPI
 *
 * @param tpi the ticks per inch of the middle encoder wheel
 *
 * <b>Example 1:</b>
 * @code
 * //using a TPI of 100
 * #define MIDDLE_TPI 100
 * @endcode
 *
* @details Sets the TPI of the middle encoder wheel.
*/
#define MIDDLE_TPI tpi          // Ticks per inch of middle wheel

// Movement tuning
/*!
 *
 * @brief Slew Step
 *
 * @param step how much to slew the motors by each time the motors are updated
 *
 * <b>Example 1:</b>
 * @code
 * //using a slew step of 10
 * #define SLEW_STEP 10
 * @endcode
 *
 * @details Sets the slew step to \a step. A smaller value results more slew.
 */
#define SLEW_STEP step         // Encoder ticks to move at each step

/*!
 * 
 * @brief Linear Exit Error
 *
 * @param error the error to use when exiting linear movement
 *
 * <b>Example 1:</b>
 * @code
 * //using an error of 4 units
 * #define LINEAR_EXIT_ERROR 4
 * @endcode
 *
 * @details Sets the error to use when exiting linear movement to \a error.
 */
#define LINEAR_EXIT_ERROR error // Error to use when exiting linear movement

/*!
 *
 * @brief Angular Exit Error
 *
 * @param error the error to use when exiting angular movement
 *
 * <b>Example 1:</b>
 * @code
 * //using an error of 4 units
 * #define ANGULAR_EXIT_ERROR 4
 * @endcode
 *
 * @details Sets the error to use when exiting angular movement to \a error.
 */
#define ANGULAR_EXIT_ERROR error // default exit distance for angular movements

/*!
 * 
 * @brief Linear Settle Threshold
 *
 * @param threshold the threshold to use when settling linear movement
 *
 * <b>Example 1:</b>
 * @code
 * //using a threshold of 1 units
 * #define LINEAR_SETTLE_THRESHOLD 1
 * @endcode
 *
 * @details Sets the threshold to use when settling linear movement to \a threshold. The robot is considered settled if
 * it does not move this many units within the duration of \ref SETTLE_TIME.
 */
#define SETTLE_THRESH_LINEAR .5      // amount of linear movement for settling

/*!
 * 
 * @brief Angular settle threshold
 *
 * @param threshold the threshold to use when settling angular movement
 *
 * <b>Example 1:</b>
 * @code
 * //using a threshold of 1 units
 * #define ANGULAR_SETTLE_THRESHOLD 1
 * @endcode
 *
 * @details Sets the threshold to use when settling angular movement to \a threshold. The robot is considered settled if
 * it does not move this many units within the duration of \ref SETTLE_TIME.
 */
#define SETTLE_THRESH_ANGULAR 1      // amount of angular movement for settling

/*!
 * 
 * @brief Settle Time
 *
 * @param time the time the chassis must be still to be considered settled
 *
 * <b>Example 1:</b>
 * @code
 * //using a settle time of 150 milliseconds
 * #define SETTLE_TIME 150
 * @endcode
 *
 * @details Sets the time the chassis must be still to be considered settled to \a time in milliseconds.
 * A high settle time may make movements take too long to complete, where as a low settle time may cause 
 * the robot to exit its movement prematurely.
 */
#define SETTLE_TIME time      // amount of time to count as settled

/*!
 *
 * @brief Linear kP
 *
 * @param kp the proportional constant for the linear motion PID controller
 *
 * <b>Example 1:</b>
 * @code
 * //using a kP of 0.5
 * #define LINEAR_KP 0.5
 * @endcode
 *
 * @details Sets the proportional constant for the linear motion PID controller to \a kp.
 */
#define LINEAR_KP kp

/*!
 *
 * @brief Linear kI
 *
 * @param ki the integral constant for the linear motion PID controller
 *
 * <b>Example 1:</b>
 * @code
 * //using a ki of 0.5
 * #define LINEAR_KI 0.5
 * @endcode
 *
 * @details Sets the integral constant for the linear motion PID controller to \a ki.
 */
#define LINEAR_KI ki

 /*!
 *
 * @brief Linear kD
 *
 * @param kd the derivative constant for the linear motion PID controller
 *
 * <b>Example 1:</b>
 * @code
 * //using a kd of 0.5
 * #define LINEAR_KD 0.5
 * @endcode
 *
 * @details Sets the derivative constant for the linear motion PID controller to \a kd.
 */
#define LINEAR_KD kd


#define TRACKING_KP 60		 // point tracking turning strength

/*!
 *
 * @brief Angular kP
 *
 * @param kp the proportional constant for the angular motion PID controller
 *
 * <b>Example 1:</b>
 * @code
 * //using a kP of 0.5
 * #define ANGULAR_KP 0.5
 * @endcode
 *
 * @details Sets the proportional constant for the angular motion PID controller to \a kp.
 */
#define ANGULAR_KP kp

 /*!
 *
 * @brief Angular kI
 *
 * @param ki the integral constant for the angular motion PID controller
 *
 * <b>Example 1:</b>
 * @code
 * //using a ki of 0.5
 * #define ANGULAR_KI 0.5
 * @endcode
 *
 * @details Sets the integral constant for the angular motion PID controller to \a ki.
 */
#define ANGULAR_KI ki

 /*!
 *
 * @brief Angular kD
 *
 * @param kd the derivative constant for the angular motion PID controller
 *
 * <b>Example 1:</b>
 * @code
 * //using a kd of 0.5
 * #define ANGULAR_KD 0.5
 * @endcode
 *
 * @details Sets the derivative constant for the angular motion PID controller to \a kd.
 */
#define ANGULAR_KD kd

/*!
 *
 * @brief Minimum Error
 *
 * @param error the minimum error for the robot to be considered at the target position
 *
 * <b>Example 1:</b>
 * @code
 * //using a minimum error of 2
 * #define MIN_ERROR 2
 * @endcode
 *
 * @details Sets the minimum error for the robot to be considered at the target position to \a error.
 */
#define MIN_ERROR error

// Auton selector configuration constants
/*!
 * 
 * @brief Auton Selector Routines
 *
 * @param auton_names the names of your autonomous routines.
 *
 * <b>Example 1:</b>
 * @code
 * //using the autons "left", "middle", "right", and "nothing"
 * #define AUTON_NAMES "left", "middle", "right", "nothing", ""
 * @endcode
 *
 * @details Sets the auton names to run. The names should be seprated by commas. The maximum number of autons is 10.
 * This is part of the autonomous selector configuration. More details can be found at \ref selector.h
 */

#define AUTONS auton_names // Names of autonomous routines, up to 10

/*!
 *
 * @brief Autonomous Selctor Hue
 *
 * @param hue the names of your autonomous routines.
 *
 * <b>Example 1:</b>
 * @code
 * //using the autons "left", "middle", "right", and "nothing"
 * #define AUTON_NAMES "left", "middle", "right", "nothing", ""
 * @endcode
 *
 * @details Sets the auton names to run. The names should be seprated by commas. The maximum number of autons is 10.
 * YOU MUST PUT AN EMPTY STRING AT THE END OF THE LIST.
 * This is part of the autonomous selector configuration. More details can be found at \ref selector.h
 */
#define HUE hue     // Color of theme from 0-359(H part of HSV)

/*!
 *
 * @brief Autonomous Selctor Default Routine
 *
 * @param index the index for the default autonomous routine to run.
 *
 * <b>Example 1:</b>
 * @code
 * //using the default auton to be "left"
 * #define DEFAULT_AUTON 1
 * @endcode
 *
 * @details Sets the default auton to run. This is part of the autonomous selector configuration. More details can be found at \ref selector.h
 */
#define DEFAULT index // Default auton selected

// Initializer
/*!
 *
 * @brief Initialize ARMS using the user defined constants
 * <b>Example 1:</b>
 * @code
 * // initalize arms
 * arms::init();
 * @endcode
 *
 * @details initialize ARMS (pid, chassis, odom, etc) using the defined constants in \ref config.h
 */
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

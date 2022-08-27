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
 * <B>YOU NEED TO REPLACE ALL OF THESE VALUES WITH THE CORRECT VALUES FOR YOUR ROBOT.</B>
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
#define ODOM_DEBUG 0

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
#define LEFT_MOTORS 0

/*!
 * @brief Right chassis motors
 *
 * 
 * @param ports the motor ports on the right side of the chassis
 * @code
 * <b> Example 1: </b>
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
#define RIGHT_MOTORS 0

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
#define GEARSET 0 // RPM of chassis motors

/*!
 * @brief Ticks per Inch
 *
 * @param tpi the number of encoder ticks per inch of forward robot movement.
 * 
 * <b>Example 1:</b>
 * @code
 * //using a tpi of 10.
 * #define TPI 10
 * @endcode
 *
 * @details Sets the number of encoder ticks per inch of forward robot movement to \a tpi.
 */
#define TPI 0      			  // Encoder ticks per inch of forward robot movement

/*!
 * @brief Middle Ticks Per Inch
 *
 * @param mtpi encoder ticks per inch of robot movement for the perpendicular middle wheel
 * 
 * <b>Example 1:</b>
 * @code
 * //using a middle tpi of 10.
 * #define MIDDLE_TPI
 * @endcode
 *
 * @details Sets the number of middle encoder ticks per inch of perpendicular robot movement to \a tpi.
 */
#define MIDDLE_TPI 0          // Ticks per inch for the middle wheel

// Tracking wheel distances
/*!
 * @brief Track Width
 *
 * @param twidth The track width of the robot (distance between left and right weels)
 * 
 * <b>Example 1:</b>
 * @code
 * //using a track width of 16 inches.
 * #define TRACK_WIDTH 16
 * @endcode
 *
 * @details Sets the number of encoder ticks per inch of forward robot movement to \a tpi.
 */
#define TRACK_WIDTH 0 		  // The distance between left and right wheels (or tracker wheels)

/*!
 * @brief Middle Distance
 *
 * @param dist the distance between the middle wheel and the center of the robot
 * 
 * <b>Example 1:</b>
 * @code
 * //using a middle distance of 2 inches.
 * #define MIDDLE_DISTANCE 2
 * @endcode
 *
 * @details 
 */
#define MIDDLE_DISTANCE 0     // Distance from middle wheel to the robot turning center

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
#define IMU_PORT 0                         // Port 0 for disabled

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
#define ENCODER_PORTS 0, 0, 0    // Port 0 for disabled

/*!
 *
 * @brief Encoder ADI Expander Port
 *
 * @param port the port the ADI expander is plugged into. 0 for disabled.
 *
 * <b>Example 1:</b>
 * @code
 * //using an ADI expander in port 1
 * #define EXPANDER_PORT 1
 * @endcode
 * <b>Example 2:</b>
 * @code
 * //don't use an ADI expander
 * #define EXPANDER_PORT 0
 * @endcode
 *
 * @details Uses the expander port in port \a port for the encoder's configured at \ref ENCODER_PORTS.
 */
#define EXPANDER_PORT 0                     // Port 0 for disabled

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
#define ENCODER_TYPE 0 // The type of encoders
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
#define SLEW_STEP 0         // Encoder ticks to move at each step

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
#define LINEAR_EXIT_ERROR 0 // Error to use when exiting linear movement

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
#define ANGULAR_EXIT_ERROR 0 // default exit distance for angular movements

/*!
 * 
 * @brief Linear Settle Threshold
 *
 * @param threshold the threshold to use when settling linear movement
 *
 * <b>Example 1:</b>
 * @code
 * //using a threshold of 1 units
 * #define SETTLE_THRESH_LINEAR 1
 * @endcode
 *
 * @details Sets the threshold to use when settling linear movement to \a threshold. The robot is considered settled if
 * it does not move this many units within the duration of \ref SETTLE_TIME.
 */
#define SETTLE_THRESH_LINEAR 0      // amount of linear movement for settling

/*!
 * 
 * @brief Angular settle threshold
 *
 * @param threshold the threshold to use when settling angular movement
 *
 * <b>Example 1:</b>
 * @code
 * //using a threshold of 1 units
 * #define SETTLE_THRESH_ANGULAR 1
 * @endcode
 *
 * @details Sets the threshold to use when settling angular movement to \a threshold. The robot is considered settled if
 * it does not move this many units within the duration of \ref SETTLE_TIME.
 */
#define SETTLE_THRESH_ANGULAR 0     // amount of angular movement for settling

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
#define SETTLE_TIME 0      // amount of time to count as settled

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
#define LINEAR_KP 0

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
#define LINEAR_KI 0

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
#define LINEAR_KD 0

/*!
 *
 * @brief Tracking kP
 *
 * @param trackkp the proportional constant for turning strength during point to point movements
 *
 * <b>Example 1:</b>
 * @code
 * //using a Tracking kP of 60
 * #define TRACKING_KP 60
 * @endcode
 *
 * @details sets the proportional constant for turning strength during point to point movements to \a trackkp
 */
#define TRACKING_KP 0		 // point tracking turning strength

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
#define ANGULAR_KP 0

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
#define ANGULAR_KI 0

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
#define ANGULAR_KD 0
#define LEAD_PCT .6			 // Go-to-pose lead distance ratio (0-1)

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
#define MIN_ERROR 0

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
 * #define AUTONS "left", "middle", "right", "nothing"
 * @endcode
 *
 * @details Sets the auton names to run. The names should be seprated by commas. The maximum number of autons is 10.
 * This is part of the autonomous selector configuration. More details can be found at \ref selector.h
 */

#define AUTONS 0 // Names of autonomous routines, up to 10

/*!
 *
 * @brief Autonomous Selctor Hue
 *
 * @param hue the hue of your autonomous selector
 * <b>Example 1:</b>
 * @code
 * //using a hue of 60 (yellow)
 * #define HUE 0
 * @endcode
 *
 * @details S
 * This is part of the autonomous selector configuration. More details can be found at \ref selector.h
 */
#define HUE 0     // Color of theme from 0-359(H part of HSV)

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
#define DEFAULT 0 // Default auton selected

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

	chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, SLEW_STEP, LINEAR_EXIT_ERROR,
	              ANGULAR_EXIT_ERROR, SETTLE_THRESH_LINEAR, SETTLE_THRESH_ANGULAR, SETTLE_TIME);

	odom::init(ODOM_DEBUG, ENCODER_TYPE, {ENCODER_PORTS}, EXPANDER_PORT, IMU_PORT,
	           TRACK_WIDTH, MIDDLE_DISTANCE, TPI,
	           MIDDLE_TPI);

	pid::init(LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, TRACKING_KP, MIN_ERROR, LEAD_PCT);

	const char* b[] = {AUTONS, ""};
	selector::init(HUE, DEFAULT, b);

}

} // namespace arms

#endif

#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "ARMS/flags.h"
#include "ARMS/point.h"
#include <memory>
#include "../api.h"

/*!
    * @namespace arms::chassis
    *
    * @details This namespace contains all of the functions and variables needed to control the chassis.
*/

namespace arms::chassis {

/*!
    * @var double maxSpeed;
    * This variable is used to set the maximum speed of the chassis.
    * 
    * @var std::shared_ptr<pros::Motor_Group> leftMotors;
    * This variable is a pointer to a Motor_Group object that contains all of the left motors.
    * 
    * @var std::shared_ptr<pros::Motor_Group> rightMotors;
    * This variable is a pointer to a Motor_Group object that contains all of the right motors.
*/
extern double maxSpeed;
extern std::shared_ptr<pros::Motor_Group> leftMotors;
extern std::shared_ptr<pros::Motor_Group> rightMotors;

/*!
    * @fn void setBrakeMode(okapi::AbstractMotor::brakeMode b)
    *
    *  Sets the chassis's brake mode
    *
    * @param brakeMode The chassis brake mode
    * 
    * <b>Example 1:</b>
    * @code
    * //set the chassis's brake mode to coast
    * chassis::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    * @endcode
    * 
    * @details Sets the chassis's brake mode to a valid \ref okapi::AbstractMotor::brakeMode.
    * Options are coast, hold, and brake.
    */
void setBrakeMode(pros::motor_brake_mode_e_t b);

/*!
    * @fn bool settled()
    *
    * @return True if the chassis is settled, false otherwise
    *   
    * <b>Example 1:</b>
    * @code
    * //wait until the chassis is settled
    * while(!chassis::settled()) {
    *    pros::delay(20);
    * }
    * @endcode
    * 
    * @details Checks if the chassis is settled.
    */
bool settled();

/*!
    * @fn void waitUntilFinished(double exit_error)
    *
    * @param exit_error The minimum error from the target point to exit the wait
    *   
    * <b>Example 1:</b>
    * @code
    * //wait for the chassis to finish a movement with a 1 inch error
    * waitUntilFinished(1);
    * @endcode
    * 
    * @details Waits for the chassis to reach a target point within a certain \a exit_error. This function should not be required, as 
    * \ref chassis::move is already a blocking function by default. However, this may need to be used if the \ref chassis::move function 
    * is called with the \ref MoveFlags::async flag.
    */
void waitUntilFinished(double exit_error);

/*!
    * @fn void move(std::vector<double> target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE)
    *
    * Perform a 2D chassis movement based on the parameters provided.
    * @param target The target point to move to.
    * @param max The maximum speed to move at.
    * @param exit_error The minimum distance from the target point to exit the movement.
    * @param lp The linear kP for the movement.
    * @param ap The angular kP for the movement.
    * @param flags The flags to use when moving the chassis.
    *   
    * <b>Example 1:</b>
    * @code
    * //move the chassis to coordinate {50, 40} at 100% max speed
    * chassis::move({50, 40}, 100);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //move the chassis to coordinate {30, 72} at 100% max speed backwards
    * chassis::move({30, 72}, 100, arms::REVERSE);
    * @endcode
    * 
    * <b>Example 3:</b>
    * @code
    * //move the chassis to the pose {48, 48, 90deg} at 75% max speed with a 2 inch exit error
    * //Movements to a pose will use our `Boomerang controller`, which uses trigonometry to move to the target point and angle
    * chassis::move({48, 48, 90}, 75, 2);
    * @endcode
    * 
    * @details Moves the chassis to a target point. Almost all parameters are optional. 
    * Technically, only the \a target parameter is required. However, it is recommended to provide the max parameter aswell so that you have control over the maximum speed of the chassis.
    *
    * The \a target parameter is a vector of doubles that represents the target point (x, y), or pose (x, y, theta). For a target point, our standard point-point odometry motion is used. For a target pose, our Boomerang controller is used. More information on these can be seen at \ref MotionControl.\n
    * The \a max parameter can be used to set the maximum speed of the movement. It will default to 100% if not provided.\n
    * The \a exit_error parameter can be used to set the minimum error from the target point to exit the movement. It will default to what is provided in \ref config.h::MIN_ERROR if not provided.\n
    * The \a lp parameter can be used to set the linear kP for the movement. It will default to what is provided in \ref config.h if not provided.\n
    * The \a ap parameter can be used to set the angular kP for the movement. It will default to what is provided in \ref config.h if not provided.\n
    * The \a flags parameter can be used to set the flags for the movement. A list of them and their descriptions can be found in the \ref MoveFlags enum.\n
    */
void move(std::vector<double> target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);

/// @cond DO_NOT_DOCUMENT
void move(std::vector<double> target, double max, double exit_error,
          MoveFlags = NONE);
void move(std::vector<double> target, double max, MoveFlags = NONE);
void move(std::vector<double> target, MoveFlags = NONE);
/// @endcond

/*!
    * @fn void move(double target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE)
    *
    * @brief move the chassis a target distance forward
    *
    * @param target The target distance to move to.
    * @param flags The flags to use when moving the chassis.
    * @param max The maximum speed to move at.
    * @param exit_error The minimum distance from the target point to exit the movement.
    * @param lp The linear kP for the movement.
    * @param ap The angular kP for the movement.
    * @param flags The flags to use when moving the chassis.
    * 
    * Almost all parameters are optional. Technically, only the \a target parameter is required. However, it is recommended to provide the max parameter aswell so that you have control over the maximum speed of the chassis.
    * 
    * <b>Example 1:</b>
    * @code
    * //move the chassis forwards 24 inches at 100% max speed
    * chassis::move(24, 100);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //move the chassis backwards 72 inches at 100% max speed 
    * chassis::move(-72, 100, arms::REVERSE);
    * @endcode
    * 
    * <b>Example 3:</b>
    * @code
    * //move the chassis forwards 48 inches at 75% max speed with a 2 inch exit error with PID disabled
    * chassis::move(48, 75, 2, arms::THRU);
    * @endcode
    * 
    * @details Moves the chassis a target distance. Almost all parameters are optional. 
    * Technically, only the \a target parameter is required. However, it is recommended to provide the max parameter aswell so that you have control over the maximum speed of the chassis.
    *
    * The \a target parameter is used to specify how far the chassis should move.
    * The \a max parameter can be used to set the maximum speed of the movement. It will default to 100% if not provided.\n
    * The \a exit_error parameter can be used to set the minimum error from the target point to exit the movement. It will default to what is provided in \ref config.h::MIN_ERROR if not provided.\n
    * The \a lp parameter can be used to set the linear kP for the movement. It will default to what is provided in \ref config.h if not provided.\n
    * The \a ap parameter can be used to set the angular kP for the movement. It will default to what is provided in \ref config.h if not provided.\n
    * The \a flags parameter can be used to set the flags for the movement. A list of them and their descriptions can be found in the \ref MoveFlags enum. The arms::RELATIVE flag is always enabled for this, as you should only use this to move a relative distance straight with the bot.\n
    */
void move(double target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);

/// @cond DO_NOT_DOCUMENT
void move(double target, double max, double exit_error,
          MoveFlags = NONE);
void move(double target, double max, MoveFlags = NONE);
void move(double target, MoveFlags = NONE);
/// @endcond

/**
 * Perform a turn movement
 */

/*!
    * @fn void turn(double target, double max, double exit_error,
          double ap, MoveFlags = NONE)
    * @brief turn the chassis a target angle
    *
    * @param target The target angle to turn to.
    * @param flags The flags to use when moving the chassis.
    * @param max The maximum speed to move at.
    * @param exit_error The minimum distance from the target point to exit the movement.
    * @param ap The angular kP for the movement.
    * @param flags The flags to use when moving the chassis.
    *   
    * Almost all parameters are optional. Technically, only the \a target parameter is required. However, it is recommended to provide the max parameter aswell so that you have control over the maximum speed of the chassis.
    * 
    * <b>Example 1:</b>
    * @code
    * //turn the chassis to face 90 degrees at 100% max speed
    * chassis::turn(90, 100);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //turn the chassis 180 degrees clockwise at 100% max speed
    * chassis::turn(-180, 100, arms::RELATIVE);
    * @endcode
    * 
    * <b>Example 3:</b>
    * @code
    * //turn the chassis to face 90 degrees at 75% max speed with a 2 degree exit error with PID disabled
    * chassis::turn(90, 75, 2, arms::THRU);
    * @endcode
    * 
    * @details Turns the chassis a target angle. Almost all parameters are optional.
    * Technically, only the \a target parameter is required. However, it is recommended to provide the max parameter aswell so that you have control over the maximum speed of the chassis.
    * 
    * The \a target parameter is a double that represents the target angle (theta). We use our PID controller to turn to the target angle. More information on this can be seen at \ref MotionControl.\n
    * The \a max parameter can be used to set the maximum speed of the movement. It will default to 100% if not provided.\n
    * The \a exit_error parameter can be used to set the minimum error from the target point to exit the movement. It will default to what is provided in \ref config.h::MIN_ERROR if not provided.\n
    * The \a ap parameter can be used to set the angular kP for the movement. It will default to what is provided in \ref config.h if not provided.\n
    * The \a flags parameter can be used to set the flags for the movement. A list of them and their descriptions can be found in the \ref MoveFlags enum.
    * 
    */

void turn(double target, double max, double exit_error, double ap,
          MoveFlags = NONE);

/// @cond DO_NOT_DOCUMENT
void turn(double target, double max, double exit_error, MoveFlags = NONE);
void turn(double target, double max, MoveFlags = NONE);
void turn(double target, MoveFlags = NONE);
/// @endcond

/*!
    * @fn void turn(Point target, double max, double exit_error,
          double ap, MoveFlags = NONE)
    * @brief turn the chassis to face a target point
    *
    * @param target The target point to turn to.
    * @param flags The flags to use when moving the chassis.
    * @param max The maximum speed to move at.
    * @param exit_error The minimum distance from the target point to exit the movement.
    * @param ap The angular kP for the movement.
    * @param flags The flags to use when moving the chassis.
    * 
    * <b>Example 1:</b>
    * @code
    * //turn the chassis to face the point (24, 24) at 100% speed
    * chassis::turn({24, 24}, 100);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //turn the chassis to face the point {72, -48} at 75% speed
    * chassis::turn({72, -48}, 75);
    * @endcode
    * 
    * <b>Example 3:</b>
    * @code
    * //turn the chassis to face the point (24, 24) at 100% speed with a 2 degree exit error with PID disabled
    * chassis::turn({24, 24}, 100, 2, arms::THRU);
    * @endcode
    * 
    * @details Turns the chassis a target angle. Almost all parameters are optional.
    * Technically, only the \a target parameter is required. However, it is recommended to provide the max parameter aswell so that you have control over the maximum speed of the chassis.
    * 
    * The \a target parameter is a Point that represents the point we want to turn to face. We use our PID controller to turn to the target angle. More information on this can be seen at \ref MotionControl.\n
    * The \a max parameter can be used to set the maximum speed of the movement. It will default to 100% if not provided.\n
    * The \a exit_error parameter can be used to set the minimum error from the target point to exit the movement. It will default to what is provided in \ref config.h::MIN_ERROR if not provided.\n
    * The \a ap parameter can be used to set the angular kP for the movement. It will default to what is provided in \ref config.h if not provided.\n
    * The \a flags parameter can be used to set the flags for the movement. A list of them and their descriptions can be found in the \ref MoveFlags enum.
    * 
    */
void turn(Point target, double max, double exit_error, double ap,
          MoveFlags = NONE);

/// @cond DO_NOT_DOCUMENT
void turn(Point target, double max, double exit_error, MoveFlags = NONE);
void turn(Point target, double max, MoveFlags = NONE);
void turn(Point target, MoveFlags = NONE);
/// @endcond


/*!
    * @fn void tank(double left, double right, bool velocity = false)
    * @brief move the chassis using tank drive
    * 
    * @param left The left side velocity or percentage (0%-100%).
    * @param right The right side velocity or percentage (0%-100%).
    * @param velocity Whether the values are velocities or percentages. Defaults to false for percentage mode.
    * 
    * <b>Example 1:</b>
    * @code
    * //move the chassis forward at 100% speed
    * chassis::tank(100, 100);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //move the chassis forward at 100 rpm
    * chassis::tank(100, 100, true);
    * @endcode
    *   
    * <b>Example 3:</b>
    * @code
    * 
    * //move the left side of the chassis forward at 100% speed, and the right side backwards at 50% speed
    * chassis::tank(100, -50);
    * @endcode
    * 
    */
void tank(double left, double right, bool velocity = false);

/*!
    *
    * @fn void arcade(double forward, double turn, bool velocity = false)
    * @brief move the chassis using arcade drive
    * 
    * @param forward The forward velocity or percentage (0%-100%).
    * @param turn The turn velocity or percentage (0%-100%).
    * @param velocity Whether the values are velocities or percentages. Defaults to false for percentage mode.
    * 
    * <b>Example 1:</b>
    * @code
    * //move the chassis forward at 100% speed
    * chassis::arcade(100, 0);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //move the chassis forward at 100 rpm
    * chassis::arcade(100, 0, true);
    * @endcode
    * 
    * <b>Example 3:</b>
    * @code
    * //move the chassis forward at 100% speed, and turn left at 50% speed
    * chassis::arcade(100, -50);
    * @endcode
    *   
    */
void arcade(double vertical, double horizontal, bool velocity = false);

/// @cond DO_NOT_DOCUMENT
void init(std::initializer_list<int8_t> leftMotors,
          std::initializer_list<int8_t> rightMotors, pros::motor_gearset_e_t gearset,
          double slew_step, double linear_exit_error, double angular_exit_error, 
          double settle_thresh_linear, double settle_thresh_angular,
          int settle_time);
/// @endcond
} // namespace arms::chassis



#endif

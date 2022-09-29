#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "ARMS/flags.h"
#include "ARMS/point.h"
#include "okapi/api.hpp"


namespace arms::chassis {

extern double maxSpeed;
extern std::shared_ptr<okapi::MotorGroup> leftMotors;
extern std::shared_ptr<okapi::MotorGroup> rightMotors;

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
void setBrakeMode(okapi::AbstractMotor::brakeMode b);

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
    * @param target The target point to move to
    * @param max The maximum speed to move at
    * @param exit_error The minimum error from the target point to exit the movement
    * @param lp The linear kP for the movement
    * @param ap The angular kP for the movement
    * @param flags The flags to use when moving the chassis
    *   
    * <b>Example 1:</b>
    * @code
    * //move the chassis to coordinate {50, 40} at 100% speed
    * chassis::move({50, 40}, 100);
    * @endcode
    * 
    * <b>Example 2:</b>
    * @code
    * //move the chassis to coordinate {30, 72} at 100% backwards
    * chassis::move({30, 72}, 100, arms::REVERSE);
    * @endcode
    * 
    * @details Moves the chassis to a target point. 
    * The \a target parameter is a vector of doubles that represents the target distance, point, or pose
    * The \a max parameter can be used to set the maximum speed of the movement
    * The \a exit_error parameter can be used to set the minimum error from the target point to exit the movement
    * The \a lp parameter can be used to set the linear kP for the movement
    * The \a ap parameter can be used to set the angular kP for the movement
    * The \a flags parameter can be used to set the flags for the movement, such as \ref MoveFlags::async
    */
void move(std::vector<double> target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);

void move(std::vector<double> target, double max, double exit_error,
          MoveFlags = NONE);
void move(std::vector<double> target, double max, MoveFlags = NONE);

/*!
    * @overload void move(std::vector<double> target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE)
    * @brief move the chassis to a target point
    *
    * @param target The target point to move to
    * @param flags The flags to use when moving the chassis
    * 
    * <b>Example 1:</b>
    * @code
    * //move the chassis to a pose 
    * chassis::move({40, 40, 90}, MoveFlags::speed(0.5));
    * @endcode
    * 
void move(std::vector<double> target, MoveFlags = NONE);

/**
 * Perform 1D chassis movement
 */
void move(double target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);
void move(double target, double max, double exit_error,
          MoveFlags = NONE);
void move(double target, double max, MoveFlags = NONE);
void move(double target, MoveFlags = NONE);

/**
 * Perform a turn movement
 */
void turn(double target, double max, double exit_error, double ap,
          MoveFlags = NONE);
void turn(double target, double max, double exit_error, MoveFlags = NONE);
void turn(double target, double max, MoveFlags = NONE);
void turn(double target, MoveFlags = NONE);

/**
 * Turn to face a point
 */
void turn(Point target, double max, double exit_error, double ap,
          MoveFlags = NONE);
void turn(Point target, double max, double exit_error, MoveFlags = NONE);
void turn(Point target, double max, MoveFlags = NONE);
void turn(Point target, MoveFlags = NONE);

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
          double slew_step, double linear_exit_error, double angular_exit_error, 
          double settle_thresh_linear, double settle_thresh_angular,
          int settle_time);

} // namespace arms::chassis

#endif

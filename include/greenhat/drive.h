#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "okapi/api.hpp"

namespace greenhat{

/**
 * Set the brake mode for all drive motors
 */
void setBrakeMode(okapi::AbstractMotor::brakeMode b);

/**
 * Reset the internal motor encoders for all drive motors
 */
void reset();

/**
 * Get the average position between the sides of the drive
 */
int drivePos();

/**
 * Get a boolean that is true if the drive motors are in motion
 */
bool isDriving();

/**
 * Delay the program until the drive motors come to rest
 */
void waitUntilSettled();

/**
 * Begin an asycronous drive movement
 */
void driveAsync(double sp, int max = 100);

/**
 * Begin an asycronous turn movement
 */
void turnAsync(double sp, int max = 100);

/**
 * Perform a drive movement and wait until settled
 */
void drive(double sp, int max = 100);

/**
 * Perform a turn movement and wait until settled
 */
void turn(double sp, int max = 100);

/**
 * Move a distance at a set voltage with no PID
 */
void fastDrive(double sp, int max = 100);

/**
 * Move for a duration at a set voltage with no PID
 */
void timeDrive(int t, int left = 100, int right = 0);

/**
 * Move for a duration at a set velocity using internal PID
 */
void velocityDrive(int t, int max = 100);

/**
 * Move the robot in an arc with a set length, radius, and speed
 */
void arcLeft(int length, double rad, int max = 100, int type = 0);

/**
 * Move the robot in an arc with a set length, radius, and speed
 */
void arcRight(int length, double rad, int max = 100, int type = 0);

/**
 * Preform a forward S shaped movement with a set length, and speed
 */
void sLeft(int arc1, int mid, int arc2, int max = 100);

/**
 * Preform a forward S shaped movement with a set length, and speed
 */
void sRight(int arc1, int mid, int arc2, int max = 100);

/**
 * Preform a backward S shaped movement with a set length, and speed
 */
void _sLeft(int arc1, int mid, int arc2, int max = 100);

/**
 * Preform a backward S shaped movement with a set length, and speed
 */
void _sRight(int arc1, int mid, int arc2, int max = 100);

/**
 * Assign a voltage to each motor on a scale of -100 to 100
 */
void tank(int left, int right);

/**
 * Assign a voltage to each motor on a scale of -100 to 100
 */
void arcade(int vertical, int horizontal);

/**
 * Start the drive tasks and calibrate the Imu if it is being used
 */
void initDrive();

}

#endif

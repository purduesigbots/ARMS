#ifndef _ARMS_CONFIG_H_
#define _ARMS_CONFIG_H_

#include "ARMS/lib.h"
#include "okapi/api.hpp"

namespace arms {

// debug
#define ODOM_DEBUG 0

// negative numbers mean reversed motor
#define LEFT_MOTORS 18, -19, 20
#define RIGHT_MOTORS -11, 12, -13
#define GEARSET 600 // rpm of chassis motors

// unit constants
#define DISTANCE_CONSTANT 44.65 // ticks per distance unit
#define DEGREE_CONSTANT 1       // ticks per degree

// slew control (autonomous only)
#define SLEW_STEP 8 // smaller number = more slew

// sensors
#define IMU_PORT 16             // port 0 for disabled
#define ENCODER_PORTS -1, -1, 0 // port 0 for disabled,
#define EXPANDER_PORT 0         // port 0 for disabled

// odometry
#define LEFT_RIGHT_DISTANCE 0 // only needed for non-imu setups
#define MIDDLE_DISTANCE 0     // only needed if using middle tracker
#define MIDDLE_TPI 41.4       // Ticks per inch
#define EXIT_ERROR 6          // default exit distance for movements

// default pid constants
#define LINEAR_KP .5
#define LINEAR_KI 0
#define LINEAR_KD 0
#define ANGULAR_KP 1
#define ANGULAR_KI 0
#define ANGULAR_KD 6
#define DIF_KP 1.2     // Keep the robot driving straight
#define FEEDFORWARD 10 // Minimum power to keep the chassis moving

// Auton selector configuration constants
#define AUTONS "Front", "Back", "Do Nothing" // Names of autonomi, up to 10
#define HUE 360   // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton numbers

// initializer
inline void init() {

	chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, DISTANCE_CONSTANT,
	              DEGREE_CONSTANT, SLEW_STEP, IMU_PORT, {ENCODER_PORTS},
	              EXPANDER_PORT, EXIT_ERROR);

	pros::delay(2000);

	if (IMU_PORT != 0) {
		odom::init(ODOM_DEBUG, LEFT_RIGHT_DISTANCE, MIDDLE_DISTANCE,
		           DISTANCE_CONSTANT, MIDDLE_TPI);
	}

	pid::init(LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI, ANGULAR_KD,
	          DIF_KP, FEEDFORWARD);

	const char* b[] = {AUTONS, ""};
	selector::init(HUE, DEFAULT, b);

	pros::delay(200); // delay while PID and odom initialize
}

} // namespace arms

#endif

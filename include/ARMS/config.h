#ifndef _ARMS_CONFIG_H_
#define _ARMS_CONFIG_H_

#include "ARMS/lib.h"
#include "okapi/api.hpp"

namespace arms {

// Debug
#define ODOM_DEBUG 0

// Negative numbers mean reversed motor
#define LEFT_MOTORS 1, 2
#define RIGHT_MOTORS -3, -4
#define GEARSET 200 // RPM of chassis motors

// Unit constants
#define DISTANCE_CONSTANT 1 // Ticks per distance unit
#define DEGREE_CONSTANT 1   // Ticks per degree (should be 1 if using an IMU)

// Sensors
#define IMU_PORT 0            // Port 0 for disabled
#define ENCODER_PORTS 0, 0, 0 // Port 0 for disabled,
#define EXPANDER_PORT 0       // Port 0 for disabled

// Odometry
#define ENCODER_TYPE arms::chassis::ENCODER_ADI // The type of encoders
#define LEFT_RIGHT_DISTANCE 0 // Distance between left and right tracking wheels
#define MIDDLE_DISTANCE 0     // Distance from middle wheel to turning center
#define MIDDLE_TPI 1          // Ticks per inch of middle wheel

// Movement tuning
#define SLEW_STEP 8  // Smaller number = more slew
#define EXIT_ERROR 1 // default exit distance for movements
#define LINEAR_KP 1
#define LINEAR_KI 0
#define LINEAR_KD 0
#define ANGULAR_KP 1
#define ANGULAR_KI 0
#define ANGULAR_KD 0
#define DIF_KP 0              // Keep the robot driving straight
#define FEEDFORWARD 10        // Minimum power to keep the chassis moving
#define ODOM_ANGLE_SCALING 60 // Scale up the angular constants for 2D movements
#define LOOKAHEAD 15          // lookahead amount for purepursuit

// Auton selector configuration constants
#define AUTONS "Front", "Back", "Do Nothing" // Names of autonomi, up to 10
#define HUE 0     // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton selected

// Initializer
inline void init() {

	chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, DISTANCE_CONSTANT,
	              DEGREE_CONSTANT, SLEW_STEP, IMU_PORT, {ENCODER_PORTS},
	              EXPANDER_PORT, EXIT_ERROR, ENCODER_TYPE);

	if (IMU_PORT != 0) {
		odom::init(ODOM_DEBUG, LEFT_RIGHT_DISTANCE, MIDDLE_DISTANCE,
		           DISTANCE_CONSTANT, MIDDLE_TPI);
	}

	pid::init(LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI, ANGULAR_KD,
	          DIF_KP, FEEDFORWARD, ODOM_ANGLE_SCALING);

	const char* b[] = {AUTONS, ""};
	selector::init(HUE, DEFAULT, b);

	purepursuit::init(LOOKAHEAD);

	pros::delay(200); // Delay while PID and odom initialize
}

} // namespace arms

#endif

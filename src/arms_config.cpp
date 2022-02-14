#include "arms/api.h"

namespace arms {

// debug
#define PID_DEBUG 0
#define ODOM_DEBUG 0

// Drivetrain configuration constants

// negative numbers mean reversed motor
#define LEFT_MOTORS 1, 2
#define RIGHT_MOTORS -3, -4
#define GEARSET 200 // rpm of chassis motors

#define DISTANCE_CONSTANT 273 // ticks per distance unit
#define DEGREE_CONSTANT 2.3   // ticks per degree

// chassis settling constants
#define SETTLE_TIME 8
#define SETTLE_THRESHOLD_LINEAR 3
#define SETTLE_THRESHOLD_ANGULAR 1

// slew control (autonomous only)
#define SLEW_STEP 8 // smaller number = more slew

// sensors
#define IMU_PORT 0            // port 0 for disabled
#define ENCODER_PORTS 0, 0, 0 // port 0 for disabled,
#define EXPANDER_PORT 0       // port 0 for disabled

// odometry
#define LEFT_RIGHT_DISTANCE 6.375 // only needed for non-imu setups
#define MIDDLE_DISTANCE 5.75      // only needed if using middle tracker
#define LEFT_RIGHT_TPI 41.4       // Ticks per inch
#define MIDDLE_TPI 41.4           // Ticks per inch
#define X_DRIVE 0     // 45 degree offset for x drives without trackers
#define EXIT_ERROR 10 // exit distance for movements

// normal pid constants
#define LINEAR_KP .3
#define LINEAR_KI 0
#define LINEAR_KD .5
#define ANGULAR_KP .8
#define ANGULAR_KI 0
#define ANGULAR_KD 3
#define DIF_KP .5

// odom point constants
#define LINEAR_POINT_KP 8
#define LINEAR_POINT_KI 0
#define LINEAR_POINT_KD 0
#define ANGULAR_POINT_KP 50
#define ANGULAR_POINT_KI 0
#define ANGULAR_POINT_KD 0

// arc movements
#define ARC_KP .05
#define ARC_SLEW_STEP 2 // smaller number = slower acceleration

// Auton selector configuration constants
#define AUTONS "Front", "Back", "Do Nothing" // Names of autonomi, up to 10
#define HUE 360   // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton numbers

// initializer
void init() {
	chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, DISTANCE_CONSTANT,
	              DEGREE_CONSTANT, SETTLE_TIME, SETTLE_THRESHOLD_LINEAR,
	              SETTLE_THRESHOLD_ANGULAR, SLEW_STEP, ARC_SLEW_STEP, IMU_PORT,
	              {ENCODER_PORTS}, EXPANDER_PORT);

	odom::init(ODOM_DEBUG, LEFT_RIGHT_DISTANCE, MIDDLE_DISTANCE, LEFT_RIGHT_TPI,
	           MIDDLE_TPI, X_DRIVE, EXIT_ERROR);

	pid::init(PID_DEBUG, LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI,
	          ANGULAR_KD, LINEAR_POINT_KP, LINEAR_POINT_KI, LINEAR_POINT_KD,
	          ANGULAR_POINT_KP, ANGULAR_POINT_KI, ANGULAR_POINT_KD, ARC_KP,
	          DIF_KP);

	const char* b[] = {AUTONS, ""};
	selector::init(HUE, DEFAULT, b);

	pros::delay(200); // delay while PID and odom initialize
}

} // namespace arms
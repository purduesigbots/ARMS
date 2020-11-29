#ifndef _CONFIG_H_
#define _CONFIG_H_

// Drivetrain configuration constants
namespace chassis {
// negative numbers mean reversed motor
#define LEFT_MOTORS -15, -18
#define RIGHT_MOTORS 17, 20
#define GEARSET 200 // rpm of chassis motors

#define DISTANCE_CONSTANT 273 // ticks per distance unit, the default is a foot
#define DEGREE_CONSTANT 2.3   // ticks per degree

// chassis settling constants
#define SETTLE_COUNT 8
#define SETTLE_THRESHOLD_LINEAR 3
#define SETTLE_THRESHOLD_ANGULAR 1

// slew control (autonomous only)
#define ACCEL_STEP 8 // smaller number = more slew
#define ARC_STEP 2   // acceleration for arcs

// sensors
#define IMU_PORT 0            // port 0 for disabled
#define ENCODER_PORTS 0, 0, 0 // port 0 for disabled
#define EXPANDER_PORT 0
} // namespace chassis

namespace odom {
#define DEBUG true
#define CHASSIS_WIDTH 12.75 // only needed for non-imu setups
#define SLEW_STEP 10        // point function slew
#define EXIT_ERROR 5        // point function exit radius in inches
} // namespace odom

namespace pid {
// normal pid constants
#define LINEAR_KP .3
#define LINEAR_KD .5
#define ANGULAR_KP .8
#define ANGULAR_KD 3
#define ARC_KP .05
#define DIF_KP .5

// odom point constants
#define LINEAR_POINT_KP 8
#define LINEAR_POINT_KD 0
#define ANGULAR_POINT_KP 50
#define ANGULAR_POINT_KD 0
} // namespace pid

// Auton selector configuration constants
namespace selector {
// Names of autonomi, up to 10
#define AUTONS "Front", "Back", "Do Nothing"
#define HUE 360   // Color of theme from 0-359(H part of HSV)
#define DEFAULT 1 // Default auton numbers
} // namespace selector

#endif

#ifndef _CONFIG_H_
#define _CONFIG_H_

namespace greenhat {

// negative numbers mean reversed motor
#define LEFT_MOTORS 5, 12
#define RIGHT_MOTORS -8, -19
#define GEARSET 200 // rpm of drive motors

#define DISTANCE_CONSTANT 273 // ticks per distance unit, the default is a foot
#define DEGREE_CONSTANT 2.3   // ticks per degree

// slew control (autonomous only)
#define ACCEL_STEP 8    // smaller number = more slew
#define DECCEL_STEP 200 // 200 = no slew
#define ARC_STEP 2      // acceleration for arcs

// pid constants
#define DRIVE_KP .3
#define DRIVE_KD .5
#define TURN_KP .8
#define TURN_KD 3
#define ARC_KP .05

#define IMU_PORT 0 // port 0 for disabled
} // namespace greenhat

#endif

#ifndef _CONFIG_H_
#define _CONFIG_H_

namespace greenhat{

//negative numbers mean reversed motor
#define LEFT_MOTORS 1, 2
#define RIGHT_MOTORS -3, -4
#define GEARSET green

#define GYRO_PORT 0 //gyro (set to 0 if not using)

#define DISTANCE_CONSTANT 273 //ticks per distance unit, the default is a foot
#define DEGREE_CONSTANT 2.3 //ticks per degree

//slew control (autonomous only)
#define ACCEL_STEP 8 //smaller number = more slew
#define DECCEL_STEP 200 //200 = no slew
#define ARC_STEP 2 // acceleration for arcs

//pid constants
#define DRIVE_KP .3
#define DRIVE_KD .5
#define TURN_KP .8
#define TURN_KD 3
#define ARC_KP .05

namespace selector{

//limit switch for auton selection
#define ADI_NAV 'D'

//limit switch for auton selection
#define CONTROLLER_NAV E_CONTROLLER_DIGITAL_RIGHT

//number of autons
#define AUTO_COUNT 4

//the names of the autons
#define AUTO_NAMES "None","Red","Blue","Skills"

}

}

#endif

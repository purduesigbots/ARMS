#ifndef _PIDA_H_
#define _PIDA_H_

#include "ARMS/config.h"
#include <array>

namespace pid {

// pid mode enums
#define ODOM_HOLO_THRU 5
#define ODOM_HOLO 4
#define ODOM 3
#define ANGULAR 2
#define LINEAR 1
#define DISABLE 0

extern int mode;

extern double linearTarget;
extern double angularTarget;
extern double vectorAngle;
extern std::array<double, 2> pointTarget;

extern double arcKP; // needs to be exposed since arcs have not been integrated
                     // into new PID format

extern double difKP; // needs to be exposed for use with chassis::fast

std::array<double, 2> linear();
std::array<double, 2> angular();
std::array<double, 2> odom();

void init(bool debug = PID_DEBUG, double linearKP = LINEAR_KP,
          double linearKI = LINEAR_KI, double linearKD = LINEAR_KD,
          double angularKP = ANGULAR_KP, double angularKI = ANGULAR_KI,
          double angularKD = ANGULAR_KD,
          double linear_pointKP = LINEAR_POINT_KP,
          double linear_pointKI = LINEAR_POINT_KI,
          double linear_pointKD = LINEAR_POINT_KD,
          double angular_pointKP = ANGULAR_POINT_KP,
          double angular_pointKI = ANGULAR_POINT_KI,
          double angular_pointKD = ANGULAR_POINT_KD, double arcKP = ARC_KP,
          double difKP = DIF_KP, double min_error = MIN_ERROR);

} // namespace pid

#endif

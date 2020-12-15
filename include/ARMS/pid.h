#ifndef _PIDA_H_
#define _PIDA_H_

#include "ARMS/config.h"
#include <array>

namespace pid {

// pid mode enums
#define GTP 3
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

double linear(bool rightSide = false);
double angular();
std::array<double, 2> gtp();

void init(bool debug = PID_DEBUG, double linearKP = LINEAR_KP,
          double linearKD = LINEAR_KD, double angularKP = ANGULAR_KP,
          double angularKD = ANGULAR_KD,
          double linear_pointKP = LINEAR_POINT_KP,
          double linear_pointKD = LINEAR_POINT_KD,
          double angular_pointKP = ANGULAR_POINT_KP,
          double angular_pointKD = ANGULAR_POINT_KD, double arcKP = ARC_KP,
          double difKP = DIF_KP);

} // namespace pid

#endif

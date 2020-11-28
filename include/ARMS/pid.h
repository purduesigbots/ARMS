#ifndef _PID_H_
#define _PID_H_

#include "ARMS/config.h"

namespace pid {

extern double linearTarget;
extern double angularTarget;
extern double vectorAngle;

extern double arcKP; // needs to be exposed since arcs have not been integrated
                     // into new PID format

extern double difKP; // needs to be exposed for use with chassis::fast

double linear(bool rightSide = false);
double angular();

void init(double linearKP = LINEAR_KP, double linearKD = LINEAR_KD,
          double angularKP = ANGULAR_KP, double angularKD = ANGULAR_KD,
          double arcKP = ARC_KP, double difKP = DIF_KP);

} // namespace pid

#endif

#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include <array>

namespace arms::pid {

// pid mode enums
#define DISABLE 0
#define TRANSLATIONAL 1
#define ANGULAR 2

extern int mode;

extern double linearKP;
extern double linearKI;
extern double linearKD;
extern double angularKP;
extern double angularKI;
extern double angularKD;

extern double angularTarget;
extern Point pointTarget;

extern bool thru;
extern bool reverse;

std::array<double, 2> translational();
std::array<double, 2> angular();

void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double minPower,
          double odomAngleScaling);

} // namespace arms::pid

#endif

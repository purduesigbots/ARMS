#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include <array>

namespace arms::pid {

// pid mode enums
#define ODOM_THRU 5
#define THRU 4
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

std::array<double, 2> linear();
std::array<double, 2> angular();
std::array<double, 2> odom();

void init(double linearKP, double linearKD, double angularKP, double angularKD,
          double arcKP, double difKP);

} // namespace arms::pid

#endif

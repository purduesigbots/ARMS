#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include <array>

namespace arms::pid {

// pid mode enums
#define ODOM_THRU 6
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
extern double dif;

std::array<double, 2> linear();
std::array<double, 2> angular();
std::array<double, 2> odom();

void init(bool debug, double linearKP, double linearKI, double linearKD,
          double angularKP, double angularKI, double angularKD,
          double linear_pointKP, double linear_pointKI, double linear_pointKD,
          double angular_pointKP, double angular_pointKI,
          double angular_pointKD, double arcKP, double difKP, double min_error,
          double difMax);

} // namespace arms::pid

#endif

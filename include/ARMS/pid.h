#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include <array>

namespace arms::pid {

// pid mode enums
#define ODOM 3
#define ANGULAR 2
#define LINEAR 1
#define DISABLE 0

extern int mode;

extern double linearKP;
extern double linearKD;
extern double angularKP;
extern double angularKD;
extern double arcKP; // needs to be exposed for arcs

extern double linearTarget;
extern double angularTarget;
extern std::array<double, 2> pointTarget;

extern int direction;
extern bool thru;

std::array<double, 2> linear();
std::array<double, 2> angular();
std::array<double, 2> odom();

void init(double linearKP, double linearKD, double angularKP, double angularKD,
          double arcKP, double difKP);

} // namespace arms::pid

#endif

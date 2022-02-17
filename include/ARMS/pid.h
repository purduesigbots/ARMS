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
extern double linearKI;
extern double linearKD;
extern double angularKP;
extern double angularKI;
extern double angularKD;

extern double linearTarget;
extern double angularTarget;
extern std::array<double, 2> pointTarget;

extern int direction;
extern bool thru;

std::array<double, 2> linear();
std::array<double, 2> angular();
std::array<double, 2> odom();

void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double difKP, double feedforward);

} // namespace arms::pid

#endif

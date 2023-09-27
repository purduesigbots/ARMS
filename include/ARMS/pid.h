#ifndef _ARMS_PID_H_
#define _ARMS_PID_H_

#include <array>
#include <mutex>
#include <atomic>

namespace arms::pid {

// pid mode enums
extern int mode;
#define DISABLE 0
#define TRANSLATIONAL 1
#define ANGULAR 2

// pid constants
extern std::atomic<double> linearKP;
extern std::atomic<double> linearKI;
extern std::atomic<double> linearKD;
extern std::atomic<double> angularKP;
extern std::atomic<double> angularKI;
extern std::atomic<double> angularKD;
extern std::atomic<double> trackingKP;
extern std::atomic<double> minError;

// integral
extern std::atomic<double> in_lin;
extern std::atomic<double> in_ang;

// targets
extern std::atomic<double> angularTarget;
extern Point pointTarget;

// flags
extern bool thru;
extern bool reverse;

extern bool canReverse;

// pid functions
std::array<double, 2> translational();
std::array<double, 2> angular();

// initializer
void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double trackingKP, double minError, double leadPct);

} // namespace arms::pid

#endif

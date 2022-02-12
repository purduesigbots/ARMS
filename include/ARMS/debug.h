#ifndef _ARMS_DEBUG_H_
#define _ARMS_DEBUG_H_

/*  
 *  This is a draft for an API to assist debugging VEX robots. The main goals
 *  of this system are:
 *      - Easy to use
 *      - Information is redrawn in the same location, rather than having the
 *        terminal scroll constantly
 *      - Allow printed variables to be grouped, where each group/subgroup
 *        is indented
 *      - Immedite mode interface to allow the user better control of which
 *        variables are printed
 *  
 *  The idea of this API faces the following problems to solve:
 *      - Multitasking poses the potential for data races. If the debug API
 *        were to use a task to accomplish the printing loop, variables from
 *        other tasks might not be updated when they are displayed.
 *      - Using a present() function called by the user probably wouldn't play
 *        nicely with user tasks. 
 *      - A user calling printing to the terminal would likely mess up the
 *        formatting of the 
 *  The idea is to have an output like the following:
 *      
 *  chassis:
 *                          Left:   Mid:    Right:
 *      motor encoders:     0.0     0.0     0.0
 *      ext. Encoders       n/a     0.0     n/a
 *                          
 *                          Tilt    Yaw     Roll
 *      imu orientation:    0.0     0.0     0.0
 * 
 *  odom:
 *      curPosition:        0.0     0.0
 */

namespace arms::debug {

/**
 *  Initializes the debugging library. 
 */
void init();

} // namespace arms::debug

#endif//_ARMS_DEBUG_H_
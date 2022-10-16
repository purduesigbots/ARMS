#ifndef _ARMS_SELECTOR_H_
#define _ARMS_SELECTOR_H_

/*!
    * @namespace arms::selector
    *
    * @details This namespace deals with setting up and displaying the autonomous selector. 
    * Almost all of it's variables and functions are private, with the exception being the \a auton variables.
    * 
    * The arms selector will handle everything in the back end, so all you need to worry about is which autonomous is selected.
    * Below is a recommended format to use the ARMS autonomous selector used by Purdue SIGBots (BLRS).
    * 
    * @code
    * // example functions for the autonomous routes
    * void redSideAuton1() {
    *     // code for red side autonomous 1
    * }
    * 
    * void redSideAuton2() {
    *     // code for red side autonomous 2
    * }
    * 
    * void blueSideAuton1() {
    *     // code for blue side autonomous 1
    * }
    * 
    * void blueSideAuton2() {
    *     // code for blue side autonomous 2
    * }
    * 
    * void skillsAuton() {
    *     // code for skills autonomous
    * }
    * 
    * 
    * // inside of main.cpp
    * void autonomous() {
    *   switch (arms::selector::auton) {
    *     case 1:
    *       runRedSideAuton1(); //runs red side autonomous 1 code
    *       break;
    *     case 2:
    *       runRedSideAuton2(); //runs red side autonomous 2 code
    *       break;
    *     case -1:
    *       runBlueSideAuton1(); //runs blue side autonomous 1 code
    *       break;
    *     case -2:
    *       runBlueSideAuton2(); //runs blue side autonomous 2 code
    *       break;
    *     case 0:
    *       runSkillsAuton(); //runs skills autonomous code
    *       break;
    *   }
    * }
    * @endcode
*/
namespace arms::selector {

/*!
    * @var int arms::selector::auton
    * This int stores the currently selected autonomous.\n
    * The value will be positive if you are using the red side, and negative if you are using the blue side.\n
    * The value will be 0 if you are using the skills autonomous.\n
*/
extern int auton;

/// @cond DO_NOT_DOCUMENT
void init(int hue, int default_auton, const char** autons);
/// @endcond

} // namespace arms::selector

#endif

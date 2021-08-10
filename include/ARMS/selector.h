#ifndef _ARMS_SELECTOR_H_
#define _ARMS_SELECTOR_H_

#include "ARMS/config.h"
#include <string>

namespace arms::selector {

extern int auton;
const char* b[] = {AUTONS, ""};

/**
 * Initializes auton selector
 */
void init(int hue = HUE, int default_auton = DEFAULT, const char** autons = b);

} // namespace arms::selector

#endif

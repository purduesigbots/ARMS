#pragma once

#include "greenhat/config.h"
#include <string>

namespace selector {

extern int auton;
const char* b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char** autons = b);

} // namespace selector

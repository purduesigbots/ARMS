#ifndef _ARMS_SELECTOR_H_
#define _ARMS_SELECTOR_H_

namespace arms::selector {

extern int auton;
void init(int hue, int default_auton, const char** autons);

void destroy();
} // namespace arms::selector

#endif

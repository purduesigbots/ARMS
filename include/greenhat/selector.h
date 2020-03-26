#ifndef _SELECTOR_H_
#define _SELECTOR_H_

namespace greenhat{

namespace selector{

//starts the selector
void init();

//returns the currently selected auto as an integer starting at zero
//and couting up towards AUTO_COUNT - 1
int get();

}

}

#endif

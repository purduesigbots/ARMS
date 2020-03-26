#include "api.h"
#include "config.h"
using namespace pros;

namespace selector{

int autoNumber = 0; // keeps track of what auton is selected

Controller selector_controller(E_CONTROLLER_MASTER);

ADIDigitalIn nav(ADI_NAV);

void task(void* parameter){
  delay(200); // prevent the bug where buttons trigger randomly on startup

  //amount and names of autons
  const char *autoNames[AUTO_COUNT] = {AUTO_NAMES};

  //auton selector
  selector_controller.print(2, 0, "%s", autoNames[autoNumber]);

  while(1){
    //display auton
    if(selector_controller.get_digital_new_press(CONTROLLER_NAV) || nav.get_new_press()){
      autoNumber++;
      if(autoNumber == AUTO_COUNT)
        autoNumber = 0;

      selector_controller.print(2,0,"                       ");
      delay(100);
      selector_controller.print(2, 0, "%s", autoNames[autoNumber]);
    }
    delay(50);
  }
}

void init(){
  Task selector_task(
    task,
    NULL,
    TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT,
    ""
  );
}

int get(){
  return autoNumber;
}

} //namespace selector

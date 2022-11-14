
# PROS With ARMS



### Introduction
ARMS is a [PROS](https://pros.cs.purdue.edu/) library that makes writing autonomous code for VEX robots a piece of cake. 

  

## Installing ARMS

1. Download the most recent [template](https://github.com/purduesigbots/ARMS/releases)

2. Run this command from terminal `pros c fetch ARMS@3.1.0.zip`

3.  `cd` into your pros project directory in your terminal

4. Apply the library to the project `pros c apply ARMS`

5. Put `#include "ARMS/api.h"` in your main.h

6. Put `#include "ARMS/config.h"` in your main.cpp

7. Call `arms::init()` in your initialize()


## Quick start guide 
### Initializing & configuring ARMS 
The file `ARMS/config.h` contains all the macros necessary to configure ARMS. There are many different macros to fine tune ARMS for your robot. Here are some examples:

* `LEFT_MOTORS` - A list of motors used for the left wheels of the chassis.
* `RIGHT_MOTORS` - A list of motors used for the right wheels of the chassis.
* `GEAR_SET`	- The gear-set that the motors of the chassis uses.
* `DISTANCE_CONSTANT` - Used to tune how far the robot travels for a given unit of distance. We tune this so that the robot travels 12 inches accurately.
* `DEGREE_CONSTANT` - Used to tune how far the robot turns for a given unit of distance. We tune this so that the robot turns 90° accurately.
* `*_KP`, `*_KI`, `*_KD` - Tunes the PID constants for linear, angular, or tracking movement.

After modifying `ARMS/config.h`, recompile and upload your project to your robot, then the changes will take effect. 
To initialize arms, simply call `arms::init()` within the initialization section of your PROS project. This will initialize ARMS with the constants defined in `ARMS/config.h`
```cpp
void initialize() {
	arms::init();
}
```
### Subsystems of ARMS
* Chassis (`chassis.h`/`chassis.cpp`) - The API and code to control the robots movement
* Odom (`odom.h`/`odom.cpp`) - The API and code to track the robots position
* Pid (`pid.h`/`pid.cpp`) - The API and code handling ARMS' PID controller

These subsystems interact with each other to allow the robot to move accurately.

### Movement Guide
After configuring ARMS, the chassis subsystem allows the user to tell the robot how to move. There are two functions that are used to control most of the robot's movements:
```cpp 
chassis::move(target, max, exit_error, lp, ap, flags)
chassis::turn(target, max, exit_error, ap, flags)
```

In both `move()` and `turn()`, only the `target` parameter is required. The other arguments are optionally used to fine tune the movement to use constants other than those specified in `ARMS/config.h`:
* max - The maximum speed that a robot moves at when performing an action.
* exit_error - Controls the distance from the target that is considered for completing the movement.
* lp - The linear P constant to use for this movement's PID control. This is only applicable to the `move()` function.
* ap - The angular P constant to use for this movement's PID control.
* flags - Flags used to modify how the movement is carried out. See the _Movement Flags_ section bellow. 

In `turn()`,  the target is an angle in degrees; however, `move()` has 3 variations for the target parameter:
* `move(12.0, ...)` - Moves the robot forward by the specified amount. In this case, 12 inches forward.
* `move({12.0, 12.0}, ...)` - Moves the robot to the specified coordinate. In this case, `(12,12)`.
* `move({12.0, 12.0, 90.0}, ...)` - moves the robot to the specified pose. The first two numbers are the coordinate to move to, and the 3rd specifies the angle the robot should face after the movement. In this case, the robot will move to the point `(12, 12)` and face 90° degrees.

#### Movement Flags:
By default, movement in ARMS is relative to where the robots position was last reset, performed using the PID controller, and blocks the calling function until the movement is finished. These behaviors can be changed by passing various flags to the movement functions:
* ASYNC - Runs the movement without blocking the calling code. This is useful if you want the robot to move while performing another non-movement action, such as raising a lift or closing a claw. Calling `chassis::waitUntilFinished()` after an asynchronous movement will then block until the movement is finished.
* THRU - Runs the movement without using the PID controller. This is useful if you want the robot to run at full speed for the entire movement. 
* RELATIVE - Performs the movement relative to the current position of the robot, rather than where the origin was last reset.
* REVERSE - Reverses the heading of the robot when moving. This is used to have the robot back up to a point rather than turn first, then move to it. 

These flags can  combined with the `|` operation. For example:
```cpp
chassis::move({12, 13}, ASYNC | THRU); 
```
will move the robot to the coordinate `(12,13)` at full speed and without blocking the auton's code. 

## In Depth Documentation
_COMING SOON_

  

# Vex Autonomous Selector

![Screenshot_of_Selector](https://user-images.githubusercontent.com/22580992/67626102-d9e1d080-f814-11e9-84cd-63a44e6a35af.png)

  

## How to use:

* Use if statements in `void autonomous() {}` to check which auton is selected, for example:

```if(selector::auton == 1){ //run auton for Front Red }```

  

* All default cases are listed below:

* selector::auton == 1 : Red Front

* selector::auton == 2 : Red Back

* selector::auton == 3 : Do Nothing

* selector::auton == -1 : Blue Front

* selector::auton == -2 : Blue Back

* selector::auton == -3 : Do Nothing

* selector::auton == 0 : Skills

  

## How to customize the selector

All configuration is done from the `ARMS/config.h` file.

```

// selector configuration

#define HUE 360 // color of theme from 0-360

#define AUTONS "Do Nothing", "Front", "Back" // names of the autonomous programs

#define DEFAULT 1 // default auton

```

*  `HUE` - Controls the color of the theme.

*  `AUTONS` - A list of every autonomous option. This list can be made any length, but may format weirdly.

*  `DEFAULT` - The auton will be selected on startup. This is required for starting a programming skills run from the controller.

  

Credit also to Sully|80508X

  

## Additional Resources

With the basics covered in the [Getting Started guide](https://arms.readthedocs.io/), your team should be able to create a competitive program for your competition robot. For people who are interested in more advanced programming such as programming skills runs, there is a lot of potential customization with this library. The following resources may interest people who want to take their programming skills further:

- [Take a C++ programming course.](https://www.codecademy.com/learn/learn-c-plus-plus)

- [Explore the PROS API](https://pros.cs.purdue.edu/v5/index.html)

- [Learn PID](http://georgegillard.com/documents/2-introduction-to-pid-controllers)

- [Read the Vex Forums a lot](http://vexforum.com)

- [Get help from other teams on discord](https://discordapp.com/invite/9JDWW8e)

# PROS With ARMS

### Introduction
ARMS is a library that makes programming a VEX V5 robot a piece of cake. 

## Installing ARMS
1. Download the most recent [template](https://github.com/purduesigbots/ARMS/releases)
2. Run this command from terminal `prosv5 c fetch ARMS@1.1.0.zip`
3. `cd` into your pros project directory in your terminal
4. Apply the library to the project `prosv5 c apply ARMS`
5. Put `#include "ARMS/api.h"` in your main.h

## Getting started
Check the Getting Started guide [in the docs](https://arms.readthedocs.io/)

# Vex Autonomous Selector
![Screenshot_of_Selector](https://user-images.githubusercontent.com/22580992/67626102-d9e1d080-f814-11e9-84cd-63a44e6a35af.png)

## How to use:
* Place `selector::init();` in `void initazlize(){}` in your main.cpp file.
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
All configuration is done from the `autoSelect/selection.h` file.
```
// selector configuration
#define HUE 360 // color of theme from 0-360
#define AUTONS "Do Nothing", "Front", "Back" // names of the autonomous programs
#define DEFAULT 1 // default auton
```
* `HUE` - Controls the color of the theme.
* `AUTONS` - A list of every autonomous option. This list can be made any length, but may format weirdly.
* `DEFAULT` - The auton will be selected on startup. This is required for starting a programming skills run from the controller.

Credit also to Sully|80508X

## Additional Resources
With the basics covered in the [Getting Started guide](https://arms.readthedocs.io/), your team should be able to create a competitive program for your competition robot. For people who are interested in more advanced programming such as programming skills runs, there is a lot of potential customization with this library. The following resources may interest people who want to take their programming skills further:
- [Take a C++ programming course.](https://www.codecademy.com/learn/learn-c-plus-plus)
- [Explore the PROS API](https://pros.cs.purdue.edu/v5/index.html)
- [Learn PID](http://georgegillard.com/documents/2-introduction-to-pid-controllers)
- [Read the Vex Forums a lot](http://vexforum.com)
- [Get help from other teams on discord](https://discordapp.com/invite/9JDWW8e)

# PROS With ARMS

### Introduction
ARMS is a library that makes programming a VEX V5 robot a piece of cake. 

## Installing ARMS
1. Download the most recent [template](https://github.com/purduesigbots/ARMS/releases)
2. Run this command from terminal `prosv5 c fetch ARMS@1.1.0.zip`
3. `cd` into your pros project directory in your terminal
4. Apply the library to the project `prosv5 c apply ARMS`
5. Put `#include "ARMS/api.h"` in your main.h
6. Put `using namespace ARMS;` in your main.h (optional)

## Getting started

### Configuring the drive
All configuration of the library will happen in the file `ARMS/config.h`. These are the settings we can use to configure how the drive motors perform. Start by configuring the motor ports according to what ports your drive motors are plugged into.
```
//negative numbers mean reversed motor
#define LEFT_MOTORS 1, 2
#define RIGHT_MOTORS -3, -4
#define GEARSET 200
```

This library was designed with the standard tank drives in mind.
If your drive is using torque or turbo motors, you need to change the gearset to `100` or `600` respectively.

If you have an X-drive, mecanum drive, an H-drive, or any other non-standard type of drive, this library won't work without a bit of a makeover. Your options are to either modify this library, or write your own drive code (sorry).

There are two default operator control modes supported, tank and arcade. Put one of the following into your opcontrol loop:
```
arcade(
  controller.get_analog(ANALOG_LEFT_Y)*(double)100/127,
  controller.get_analog(ANALOG_LEFT_X)*(double)100/127
);

tank(
  controller.get_analog(ANALOG_LEFT_Y)*(double)100/127,
  controller.get_analog(ANALOG_LEFT_X)*(double)100/127
);
```
In these examples, I am using the pros::controller, but you can use the okapi controller as well. Just scale the value between -100 and 100

## Autonomous movement
With the drive motors properly configured, we can now control our robot’s movement autonomously. The drive function interprets positive numbers such as `2.5` as forward movement, and it interprets negative numbers such as `-0.8` as backwards movement. Similarly, the turn function interpret positive numbers as a left turn, and negative numbers as a right turn.

Here is an example of some common autonomous movements you may perform
```
void redAuton(){
  drive(2);
  turn(90);
  drive(.5);
  turn(-60);
  drive(-1);
}
```


### Tuning the drive
The above drive movements probably won't work right out of the box. To fix that you might need to make a few adjustments.
Near the top of config.h there are two important constants we need to adjust.
```
#define DISTANCE_CONSTANT 273 //ticks per distance unit, the default is a foot
#define DEGREE_CONSTANT 2.3 //ticks per degree
```

`DISTANCE_CONSTANT` correlates to the `drive()` function and `DEGREE_CONSTANT` correlates to the `turn()` function.

To tune the constants, tell the robot to drive 1 unit in autonomous.
```
drive(1)
```  
Adjust the distance constant until 1 unit equals whatever real life equivalent. When you tell the robot to `drive(2.5);`, the distance constant determines if the robot drives 2.5 meters, 2.5 feet, 2.5 inches, or some other distance.

If the robot was overshooting the target distance, then reduce the distance constant and try again. If the robot was undershooting, increase the distance constant. Don’t forget to re-upload the program each time you change the constant.

The next thing to tune is the turns. Set the robot to turn 90 degrees in autonomous
```
turn(90);
```
Go through the same trial and error process, except this time change the degree constant instead of the distance constant. A good way to test if your robot is turning exactly 90 is to run the auton four times and see if the robot does a near-perfect 360. If it's within a couple degrees, it's good enough for most autons.


### Slowing it down
By default, all robot movements are top speed. This is usually fine, but occasionally the robot can be made more accurate by slowing down its movements.
The drive functions all have an optional parameter that allows us to set a maximum speed.
```
drive(2, 50);
```
The second number after the comma is the new top speed on a scale of 0-100. In the above example, it would run the robot at half speed. This is nice for picking up game objects, precise maneuvers, and lining up with walls.


### Speeding it up
Sometimes when driving, you may want to drive a certain distance, then have a new action happen while in motion.
For instance, we could drive 2 feet, then start running our intake, and keep driving another 1 foot. If we did this with the regular drive function it would look like this.
```
drive(2);
intake.move(127);
drive(1);
intake.move(0);
```
The problem with this method is that the robot will come to a complete stop before it starts running the intake. If we want the robot to keep driving without stopping we need to use the function `fastDrive`. This will keep the robot from stopping. Here is an example:
```
fastDrive(2);
intake.move(127);
drive(1);
intake.move(0);
```

The fast drive function can also be used to slow down the robot when approaching an object too fast. For example, if we want to pick up a game object that is 3 feet away from the robot, we might drive the first 2 feet at top speed, then slow down as we approach the game object.
```
fastDrive(2);
intake.move(127);
drive(1, 50);
```
In this example, we can also avoid running the intake until we get close to the game object. This is especially useful for claw intakes which cannot start to close until the robot is close to what it is trying to pick up.

### Hold mode
Some drivers like to have their drive motors on hold mode. This prevents the robot from being pushed around as much and generally reduces the effects of defense.
To achieve this effect call the function `setBrakeMode(okapi::AbstractMotor::brakeMode::hold);` at the top of your opcontrol function.

### Time based drive movements
In autonomous, if you want to move the drive for duration of time rather than a distance, you can use the `timeDrive()` function.
`timeDrive(1000, 50);`
The first values is the duration of the movement in milliseconds, and the second values is the top speed. It is optional and defaults to 100 when omitted.

If you want to send different power to each side of the drive, call the function with two separate speeds.
`timeDrive(1000, 50, 80);`
This will produce a curved movement.

## Fancy movement
The previously covered drive movements will be good enough for most applications, but to increase the speed of the program without sacrificing consistency, there are few advanced functions of the library detailed below.

### Asynchronous drive movement
To start performing a drive movement asynchronously, use the following functions:
`driveAsync(3)`
`turnAsync(90)`
These functions will not block the program during their execution so you can perform other tasks while in motion.
Here is an example
```
driveAsync(3);
intake.move_absolute(180, 200);
waitUntilSettled();
```

### Arc turns
Driving in straight lines and turning on a dime are the two most essential things for a robot to be able to do in autonomous, however it can often be faster to do non-linear movements.
To preform an arc turn, use one of the following functions:
`arcLeft(1000, 0.5);`
`arcRight(1000, 0.5);`
The first value is the arc length in milliseconds. These movements are controlled with by velocity PID over a given time, so to make the arc longer, the time increases
The second value is the radius of the movement. This in no way correlates to measurable value, it simply is the ratio between the fast and slow side of the drive.
The radius value should be less than 1 or the movement will be a straight line. Making the movement zero will lock one side of the drive and turn with the other.
WARNING, these movements are more subject to changes based on the weight of the bot or charge of the battery than PID movements. They still are pretty consistent, but note that they can be finicky.

You can pass a max value to these movements as well. `arcLeft(1000, 0.5, 50)`
You will likely want to do this to increase consistency. On all omni wheel drives the robot can drift through the turn, which is cool, but often impracticle.

### S-Curves
These are the most efficient way to move a robot in two dimensions. To do an S-shaped movement would be slow and take several `drive()` and `turn()`
The S-curve can move the robot forward/backward, and latterally at the same time.
To preform one, use on of the following functions:
```
sLeft(800, 200, 1200);
sRight(800, 200, 1200);
```
S-curves in ARMS are made up of 3 parts.
1. The first arc - The first argument
2. Them mid movement - The second argument
3. The final arc - The third argument

The final arc length is longer than the first arc length because the robot should be slowing down during this part of the arc.
Although this may change depending on the length of the arc and the speed of the robot.

These three arguments are all in milliseconds, so we can determine the movement will take about two seconds from start to finish.
A maximum speed can (and probably should) be passed in to the function as well.
`sLeft(800, 200, 1200, 50);`
Limiting the speed will also make the whole movement much smaller due to the fact that the robot will be covering less distance.
This means that changing the max speed of an s-curve or arc turn will significantly change the movement.

To reverse an arc turn, put an underscore before the function: `_sLeft(800, 200, 1200, 50);` Voila, backwards movement.

### Custom S-curves
If the built-in s-curves are incapable of achieving the movement you desire, you can always program the three parts of an S-curve seperatly
```
arcLeft(800, 1, 50, 1);
velocityDrive(200);
arcRight(1600, 1, 50, 3);
```

The fourth arguments in the arc function calls determine the type of arc.
The first arc must be of type 1 and the last arc must be of type 3 to get the s-curve shape.
Paths can be made more complicated by adding more than 2 arc movements.
```
void redAuton(){
  arcRight(1000, 1, 50, 1);
  arcLeft(1800, 1, 50, 2);
  arcRight(1600, 1, 50, 3);
}
```
This movement will be more of a U shape. This is useful for navigating around an obstacle in one continuous movement.
All arcs performed before the first and last arc must be type 2.

# Vex Autonomous Selector
![Screenshot_of_Selector](https://user-images.githubusercontent.com/22580992/67626102-d9e1d080-f814-11e9-84cd-63a44e6a35af.png)

## How to use:
2. Place `selector::init();` in `void initazlize(){}` in your main.cpp file.
3. Make if statements in your void `void autonomous() {}` like so:

  `if(selector::auton == 1){ //run auton for Front Red }`

7. Create multiple if statements for all possible numbers of selector::auton.
8. All default cases are listed below:

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

## Where to go from here
With the basics covered, your team should be able to create a competitive program for your competition robot. For people who are interested in more advanced programming such as programming skills runs, there is a lot of potential customization with this library. People who want to take their programming skills further do the following:
- Take a C++ programming course. https://www.codecademy.com/learn/learn-c-plus-plus
- Explore the PROS api: https://pros.cs.purdue.edu/v5/index.html
- Learn PID http://georgegillard.com/documents/2-introduction-to-pid-controllers
- Read the Vex Forums a lot http://vexforum.com
- Get help from other teams on discord https://discordapp.com/invite/9JDWW8e

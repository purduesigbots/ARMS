# Getting Started

## Installing ARMS
1. Download the most recent [template](https://github.com/Marsgate/greenhatlib/releases)
2. Run this terminal command: `prosv5 c fetch greenhat@1.1.0.zip` (replace with your version number)
3. `cd` into your pros project directory in your terminal
4. Apply the library to the project using `prosv5 c apply greenhat`
5. Add `#include "greenhat/api.h"` to your `main.h`
6. (Optional) Add `using namespace greenhat;` to your `main.h`

## Configuring the Drive
All configuration settings for controlling drive motors exist in `greenhat/config.h`. Start by configuring the motor ports according to your bot's drive motors:
```c
//negative numbers mean reversed motor
#define LEFT_MOTORS 1, 2
#define RIGHT_MOTORS -3, -4
#define GEARSET 200
```

If your drive is using torque or turbo motors, you should change the gearset to `100` or `600` respectively.

## Operator Control
There are two default operator control modes supported, tank and arcade. Include one of the following in your opcontrol loop:
```cpp
arcade(
    controller.get_analog(ANALOG_LEFT_Y)*(double)100.0/127,
    controller.get_analog(ANALOG_LEFT_X)*(double)100.0/127
);

tank(
    controller.get_analog(ANALOG_LEFT_Y)*(double)100.0/127,
    controller.get_analog(ANALOG_LEFT_X)*(double)100.0/127
);
```

This example uses the values from a pros::controller object, but okapi controllers may be used as well, so long as the inputs to arcade() or tank() are scaled between -100 and 100.

### Hold Mode
Some drivers prefer their drive motors be set to hold mode. This prevents the robot from being pushed around as much and generally reduces the effects of defense.  
To achieve this effect, call the function `setBrakeMode(okapi::AbstractMotor::brakeMode::hold);` at the top of your opcontrol function.

## Basic Autonomous Movement
After the drive motors have been configured, autonomous control of the robot is possible. Two integral functions for autonomous movement are `drive()` and `turn()`. `drive()` takes a distance parameter, positive or negative, corresponding to forward or backward movement. `turn()` takes an angle parameter, positive corresponding to a left turn and negative to a right turn.  
Here is an example of these functions in use:
```cpp
void redAuton() {
    drive(2); //drive forward 2ft
    turn(90); //turn 90 degrees left
    drive(0.5);
    turn(-60); //turn 60 degrees right
    drive(-1); //drive backward 1ft
}
```

### Tuning the Drive
The above drive movements may not exactly correspond to the distances given in comments without proper tuning. To fix this tuning, there are two important constants to adjust near the top of `config.h`:
```cpp
#define DISTANCE_CONSTANT 273 //ticks per distance unit, default is a foot
#define DEGREE_CONSTANT 2.3 //ticks per degree
```
`DISTANCE_CONSTANT` corresponds to `drive()`, and `DEGREE_CONSTANT` corresponds to `turn()`. One method of tuning `drive()` is as follows:

- Instruct the robot to drive forward one unit in autonomous: `drive(1);`
- Measure the distance travelled, and compare it with your unit of choice (feet, meters, inches, etc.)
- If the bot is overshooting your target distance, reduce the distance constant.  
If undershooting, increase the distance constant.
- Repeat tests until the `drive()` function is calibrated to your desired unit. Be sure to re-upload your code each time you adjust `DISTANCE_CONSTANT`!

The same trial-and-error process can be used to tune `turn()`, adjusting `DEGREE_CONSTANT` after testing a 90-degree turn. One useful final test after a good `DEGREE_CONSTANT` value is found is to run 4 consecutive 90-degree turns to see if the robot does a full 360. If this test passes within a few degrees of error, it should be good enough for most autonomous programs.

### Slowing Down
By default, all robot movements are top speed. This is usually fine, but occasionally the robot can be made more accurate by slowing down its movements.  
The `drive()` function accepts an optional parameter to set a maximum speed:
```cpp
drive(2, 50);
```
The second parameter sets the top speed on a scale of 0-100. In the above example, the robot would drive at half speed. This is useful for picking up game objects, lining up with walls, and other precise maneuvers.

### Speeding Up
Sometimes when driving, you may want to drive a certain distance, then have a new action happen while in motion.
For instance, you could drive 2 feet, start running the intake, and keep driving another 1 foot. Doing this with the regular drive function looks like this:
```cpp
drive(2);
intake.move(127);
drive(1);
intake.move(0);
```
The problem with this method is that the robot will come to a complete stop before it starts running the intake. To ensure the robot to keeps driving without stopping, use the function `fastDrive`. Here is an example:
```cpp
fastDrive(2);
intake.move(127);
drive(1);
intake.move(0);
```

The fast drive function can also be used to slow down the robot when approaching an object too fast. For example, to pick up a game object that is 3 feet away from the robot, you might drive the first 2 feet at top speed, then slow down as you approach the game object:
```
fastDrive(2);
intake.move(127);
drive(1, 50);
```
In this example, you can also avoid running the intake until close to the game object. This is especially useful for claw intakes which cannot start to close until the robot is close to what it is trying to pick up.  
sed drive movements
In autonomous, if you want to move the drive for duration of time rather than a distance, you can use the `timeDrive()` function.
`timeDrive(1000, 50);`
The first values is the duration of the movement in milliseconds, and the second values is the top speed. It is optional and defaults to 100 when omitted.

If you want to send different power to each side of the drive, call the function with two separate speeds.
`timeDrive(1000, 50, 80);`
This will produce a curved movement.

## Advanced Autonomous Movement
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
S-curves in greenhat are made up of 3 parts.
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
```cpp
void redAuton(){
    arcRight(1000, 1, 50, 1);
    arcLeft(1800, 1, 50, 2);
    arcRight(1600, 1, 50, 3);
}
```
This movement will be more of a U shape. This is useful for navigating around an obstacle in one continuous movement.
All arcs performed before the first and last arc must be type 2.

## Where to go from here
With the basics covered, your team should be able to create a competitive program for your competition robot. For people who are interested in more advanced programming such as programming skills runs, there is a lot of potential customization with this library. People who want to take their programming skills further do the following:

- Take a C++ programming course. https://www.codecademy.com/learn/learn-c-plus-plus
- Explore the PROS api: https://pros.cs.purdue.edu/v5/index.html
- Learn PID http://georgegillard.com/documents/2-introduction-to-pid-controllers
- Read the Vex Forums a lot http://vexforum.com
- Get help from other teams on discord https://discordapp.com/invite/9JDWW8e


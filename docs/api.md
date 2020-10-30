# ARMS API
TO DO:  

- clarify meaning of slew and PID constants
- better differentiate use cases of asynchronous vs regular drive and turn functions
- clarify meaning of arc types 0, 1, 2, 3
- confirm that arc left means CCW

## Constants

### Chassis
| Constant | Default | Description |
| -------- | :----------: | -------------- |
| LEFT_MOTORS | 1, 2 | The port numbers (1-21) of the left motors of the drive. A negative number will run the motor in reverse (clockwise). |
| RIGHT_MOTORS | -3, -4 | The port numbers (1-21) of the right motors of the drive. A negative number will run the motor in reverse (clockwise). |
| GEARSET | 200 | The RPM of the drive motors. |
| DISTANCE_CONSTANT | 273 | Number of motor encoder ticks per distance unit (typically feet) used in driving functions. Default value corresponds to a medium-sized robot driving 1 foot. |
| DEGREE_CONSTANT | 2.3 | Number of motor encoder ticks per degree of rotation used in turning functions. Default value corresponds to a medium-sized robot rotating 1 degree. |
| ACCEL_STEP | 8 | Used in slew control; a smaller number refers to more slew |
| DECEL_STEP | 200 | Used in slew control; default value of 200 means no slew |
| ARC_STEP | 2 | Used in slew control |
| DRIVE_KP | 0.3 | Used in PID control |
| DRIVE_KD | 0.5 | Used in PID control |
| TURN_KP | 0.8 | Used in PID control |
| TURN_KD | 3 | Used in PID control |
| ARC_KP | 0.05 | Used in PID control |
| IMU_PORT | 0 | The port number (1-21) of the Internal Measurement Unit (IMU). Value of 0 means no IMU. |

### Selection
| Constant | Default Value | Description |
| -------- | :----------: | -------------- |
| DEFAULT | 1 | The auton number of the default auton. Auton numbers start from 1. |
| HUE | 360 | The base color of the selection screen as a hue (0-360). Default 360 is pure red. |
| AUTONS | "Front", "Back", "Do Nothing" | The auton names to be displayed on the selection screen. Up to 10 autons are accepted. |

## Functions

### Chassis
**setBrakeMode**
```cpp
void setBrakeMode(okapi::AbstractMotor::brakeMode b);
```
`b` - The Okapi brake mode (coast = 0, brake = 1, hold = 2, invalid = INT32_MAX)  

Sets the brake mode of the drive motors.
```cpp
setBrakeMode(0); // set drive motors to coast mode
```

**reset**
```cpp
void reset();
```
Resets the motor encoders of the drive motors, so that their current position is considered zero.  

**drivePos**
```cpp
int drivePos();
```
Returns the average position between the left and right drive motors.

**isDriving**
```cpp
bool isDriving());
```
Returns True if the robot is driving (i.e. the drive motors are in motion), or false if they are not.

**waitUntilSettled**
```cpp
void waitUntilSettled();
```
Delays the program until the robot is no longer driving (see isDriving).

**driveAsync**
```cpp
void driveAsync(double sp, int max = 100)
```
`sp` - The target driving distance (default units of feet)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Performs a driving action asynchronously, meaning that the robot does not wait until settled upon reaching the destination.
```cpp
driveAsync(2); // drive 2 feet forward
driveAsync(-2, 50); // drive 2 feet backward, topping out at half motor speed
```

**turnAsync**
```cpp
void turnAsync(double sp, int max = 100)
```
`sp` - The target turning distance (degrees): positive is counterclockwise  
`max` - The maximum motor speed during the turning action (scaled from 0-100, default 100)

Performs a turning action asynchronously, meaning that the robot does not wait until settled upon reaching the destination.
```cpp
turnAsync(90); // turn 90 degrees CCW
turnAsync(-90, 50); // turn 90 degrees CW, topping out at half motor speed
```

**drive**
```cpp
void drive(double sp, int max = 100)
```
`sp` - The target driving distance (default units of feet)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Performs a driving action and waits until the robot is settled (see waitUntilSettled) upon reaching the destination.
```cpp
drive(2); // drive 2 feet forward
drive(-2, 50); // drive 2 feet backward, topping out at half motor speed
```

**turn**
```cpp
void turn(double sp, int max = 100)
```
`sp` - The target turning distance (degrees): positive is counterclockwise  
`max` - The maximum motor speed during the turning action (scaled from 0-100, default 100)

Performs a turning action and waits until the robot is settled (see waitUntilSettled) upon reaching the destination.
```cpp
turn(90); // turn 90 degrees CCW
turn(-90, 50); // turn 90 degrees CW, topping out at half motor speed
```

**fastDrive**
```cpp
void fastDrive(double sp, int max = 100)
```
`sp` - The target driving distance (default units of feet)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Performs a driving action with no PID or waiting until settled. That is, the drive motors are immediately set to max in the designated direction.

**timeDrive**
```cpp
void timeDrive(int t, int left = 100, int right = 0);
```
`t` - The duration of the drive action (ms)  
`left` - The left motor speed during the driving action (scaled from 0-100, default 100)  
`right` - The right motor speed during the driving action (scaled from 0-100): if equal to zero, use left motor speed

Drives with no PID (drive motors are immediately set to given values) for a specified duration. Specific motor speeds scaled from 0-100 can also be provided.
```cpp
timeDrive(1000); // drive forward for 1 second at max speed
timeDrive(2000, 50); // drive forward for 2 seconds at half speed
timeDrive(500, 100, 50); // drive and turn slightly right for 0.5 seconds
```

**velocityDrive**
```cpp
void velocityDrive(int t, int max = 100);
```
`t` - The duration of the drive action (ms)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Drives with internal PID (see left_drive_vel) in a straight line for a specified duration.  
```cpp
velocityDrive(1000); // drive forward for 1 second at max speed
velocityDrive(2000, 50); // drive forward for 2 seconds at half speed
```

**arcLeft**
```cpp
void arcLeft(int length, double rad, int max = 100, int type = 0);
```
`length` - The desired arc length (degrees)  
`rad` - The radius of the arc (default units of feet)  
`max` - The maximum motor speed during the arc action (scaled from 0-100, default 100)  
`type` - The type of arc (default 0)

Drives in a counterclockwise arc with specified length, radius, and type.
```cpp
arcLeft(180, 1); // traces semicircle with radius 1 ft in the CCW direction
arcLeft(90, 2, 50); // traces quarter circle with radius 2 ft in CCW direction at half speed
```

**arcRight**
```cpp
void arcRight(int length, double rad, int max = 100, int type = 0);
```
`length` - The desired arc length (degrees)  
`rad` - The radius of the arc (default units of feet)  
`max` - The maximum motor speed during the arc action (scaled from 0-100, default 100)  
`type` - The type of arc (default 0)  

Drives in a clockwise arc with specified length, radius, and type.
```cpp
arcRight(180, 1); // traces semicircle with radius 1 ft in the CW direction
arcRight(90, 2, 50); // traces quarter circle with radius 2 ft in CW direction at half speed
```

**function_name**
```cpp
void function_name(var1, var2);
```
`var1` - The first variable.  
`var2` - The second variable.  

A description of what the function does, including an example if necessary.  
```cpp
example code
```

### Selection

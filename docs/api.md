# ARMS API

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
| ARC_STEP | 2 | Used in slew control |
| IMU_PORT | 0 | The port number (1-21) of the Internal Measurement Unit (IMU). A value of 0 means no IMU. |
| ENCODER_PORTS | 0, 0, 0 | The port numbers (1-8) of the first wire of up to 3 Vex Optical Shafter Encoders. The order of the ports is left encoder, right encoder, middle encoder. A value of 0 means no encoder is connected. |
| EXPANDER_PORT | 0 | The port number (1-21) of the vex 3-wire port expander. A value of 0 means no port expander. |
| JOYSTICK_THRESHOLD | 10 | The amount (0-100) that the joystick has to move before sending power to the motors. |

### Odometry
| ODOM_DEBUG | 0 | A boolean value that when set to 1 will print the current odom position to the terminal. |
| LEFT_RIGHT_DISTANCE | 6.375 | The distance in inches from the left/right tracking wheels to the center of robot rotation. This is not needed if an IMU is being used. |
| MIDDLE_DISTANCE | 5.75 | The distance in inches from the middle tracking wheel to the center of robot rotation. This is only needed if a middle tracking wheel is being used. |
| LEFT_RIGHT_TPI | 41.4 | The amount of encoder ticks required to move the left/right tracking wheels 1 inch. |
| MIDDLE_TPI | 41.4 | The amount of encoder ticks required to move the middle tracking wheel 1 inch. |
| SLEW_STEP | 10 | The amount of slew for the point-to-point movement functions. A lower number will accelerate more slowly. |
| HOLONOMIC | 0 | A boolean value that indicates that the chassis being used is an X-drive. |
| EXIT_ERROR | 10 | The exit distance for moveThru and holoThru movements. |


### PID
| PID_DEBUG | 0 | A boolean value that when set to 1 will print PID information to the terminal.
| LINEAR_KP | 0.3 | Used in PID control, scales linearly with distance between current position and target (helps when starting moves). |
| LINEAR_KI | 0.3 | Used in PID control, scales based on steady state error (helps when ending movements). |
| LINEAR_KD | 0.5 | Used in PID control, scales based on the change in distance between two calculation steps (helps to smooth motion and avoid overshoot, but dampens acceleration). |
| ANGULAR_KP | 0.8 | Used in PID control for turns. |
| ANGULAR_KI | 0.8 | Used in PID control for turns. |
| ANGULAR_KD | 3 | Used in PID control for turns. |
| ARC_KP | 0.05 | Used in PID control for arc moves. |
| DIF_KP | 0.5 | Used in PID control to help keep the robot moving in a straight line. |
| LINEAR_POINT_KP | 8 | Linear KP for point-to-point odometric movement. |
| LINEAR_POINT_KI | 0 | Linear KI for point-to-point odometric movement. |
| LINEAR_POINT_KD | 0 | Linear KD for point-to-point odometric movement. |
| ANGULAR_POINT_KP | 50 | Angular KP for point-to-point odometric movement. |
| ANGULAR_POINT_KI | 0 | Angular KI for point-to-point odometric movement. |
| ANGULAR_POINT_KD | 0 | Angular KD for point-to-point odometric movement. |
| MIN_ERROR | 5 | Minimum error before the robot stops attempting to turn to face the target point. This prevents the robot from spinning around the target without ever reaching it. |


### Selection
| Constant | Default Value | Description |
| -------- | :----------: | -------------- |
| DEFAULT | 1 | The auton number of the default auton. Auton numbers start from 1. |
| HUE | 360 | The base color of the selection screen as a hue (0-360). Default 360 is pure red. |
| AUTONS | "Front", "Back", "Do Nothing" | The auton names to be displayed on the selection screen. Up to 10 autons are accepted. |

# Chassis Functions

##**setBrakeMode**
```cpp
void setBrakeMode(okapi::AbstractMotor::brakeMode b);
```
`b` - The Okapi brake mode (coast = 0, brake = 1, hold = 2, invalid = INT32_MAX)  

Sets the brake mode of the drive motors.
```cpp
setBrakeMode(0); // set drive motors to coast mode
```
___
##**resetAngle**
```cpp
void resetAngle(double angle = 0);
```
Resets the IMU angle to the target angle in degrees.
___
##**reset**
```cpp
void reset();
```
Resets the motor encoders of the drive motors, so that their current position is considered zero.  
___
##**position**
```cpp
double position(bool yDirection = false, bool forceEncoder = false);
```
Returns the position, as averaged between the left and right drive motors.
___
##**angle**
```cpp
double angle();
```
Returns the angle of the chassis in degrees.
___
##**difference**
```cpp
double difference();
```
Returns the difference in position between the left and right sides of the chassis.
___
##**limitSpeed**
```cpp
double limitSpeed(double speed, double max);
```
Reduce an input speed to within the bounds of a given max.
___
##**slew**
```cpp
double slew(double speed, double step, double* prev);
```
Returns a gradually accelerating speed towards the target input. Must be called in a loop. The 'prev' variable must persist between function calls.
___
##**settled**
```cpp
bool settled();
```
Returns false if the robot is driving (i.e. the drive motors are in motion), or true if they are not.
___
##**waitUntilSettled**
```cpp
void waitUntilSettled();
```
Delays the program until the robot is no longer driving (see settled).
___
##**moveAsync**
```cpp
void moveAsync(double sp, int max = 100)
```
`sp` - The target driving distance (default units of inches, must be manually calibrated, see [DISTANCE_CONSTANT](#constants))  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Performs a driving action asynchronously, meaning that the robot does not wait until settled upon reaching the destination.
```cpp
driveAsync(24); // drive 24 inches forward
driveAsync(-24, 50); // drive 24 inches backward, topping out at half motor speed
```
___
##**turnAsync**
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
___
##**turnAbsoluteAsync**
```cpp
void turnAbsoluteAsync(double sp, int max = 100)
```
`sp` - The target turning distance (degrees): positive is counterclockwise  
`max` - The maximum motor speed during the turning action (scaled from 0-100, default 100)

Performs a turning action to an absolute angle asynchronously, meaning that the robot does not wait until settled upon reaching the destination. The angle is retrieved from the IMU.
___
##**holoAsync**
```cpp
void holoAsync(double distance, double angle, int max = 100)
```
`distance` - The target driving distance (default units of inches).
`angle` - The target angle relative to forward.
`max` - The maximum motor speed during the action (scaled from 0-100, default 100)

Performs a holonomic movement. This is the equivalent to `moveAsync` but by changing the angle parameter, the robot can move at odd angles and perform strafing movements.
___
##**move**
```cpp
void move(double sp, int max = 100)
```
`sp` - The target driving distance (default units of inches)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Performs a driving action and waits until the robot is settled (see waitUntilSettled) upon reaching the destination.
```cpp
drive(24); // drive 24 inches forward
drive(-24, 50); // drive 24 inches backward, topping out at half motor speed
```
___
##**turn**
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
___
##**turnAbsolute**
```cpp
void turn(double sp, int max = 100)
```
`sp` - The target turning distance (degrees): positive is counterclockwise  
`max` - The maximum motor speed during the turning action (scaled from 0-100, default 100)

Performs a turning action to an absolute angle and waits until the robot is settled (see waitUntilSettled) upon reaching the destination. The angle is retrieved from the IMU.
```cpp
turn(90); // turn 90 degrees CCW
turn(-90, 50); // turn 90 degrees CW, topping out at half motor speed
```
___
##**fast**
```cpp
void fast(double sp, int max = 100)
```
`sp` - The target driving distance (default units of inches)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Performs a driving action with no PID or waiting until settled. That is, the drive motors are immediately set to max in the designated direction.  
Useful for wall alignment or other forced moves.
___
##**voltage**
```cpp
void voltage(int t, int left_speed = 100, int right_speed = 0);
```
`t` - The duration of the drive action (ms)  
`left_speed` - The left motor speed during the driving action (scaled from 0-100, default 100)  
`right_speed` - The right motor speed during the driving action (scaled from 0-100): if equal to zero, use left motor speed

Drives with no PID (drive motors are immediately set to given values) for a specified duration. Specific motor speeds scaled from 0-100 can also be provided.
```cpp
timeDrive(1000); // drive forward for 1 second at max speed
timeDrive(2000, 50); // drive forward for 2 seconds at half speed
timeDrive(500, 100, 50); // drive and turn slightly right for 0.5 seconds
```
___
##**velocity**
```cpp
void velocity(int t, int max = 100);
```
`t` - The duration of the drive action (ms)  
`max` - The maximum motor speed during the driving action (scaled from 0-100, default 100)

Drives with the motors' **BUILT-IN** PID Control (see left_drive_vel) in a straight line for a specified duration.  
```cpp
velocity(1000); // drive forward for 1 second at max speed
velocity(2000, 50); // drive forward for 2 seconds at half speed
```
___
##**arcLeft/arcRight**
```cpp
void arcLeft(int length, double rad, int max = 100, int type = 0);
void arcRight(int length, double rad, int max = 100, int type = 0);
```
`length` - The desired arc length (degrees)  
`rad` - The radius of the arc (default units of feet)  
`max` - The maximum motor speed during the arc action (scaled from 0-100, default 100)  
`type` - The type of arc (default 0)

Drives in a counterclockwise arc with specified length, radius, and type.  
The motivation behind arcMove types is to enable custom curved movement, such that doing arc moves with subsequent types would produce one continuous move:
```cpp
arcLeft(90, 1, 100, 1) // traces a quarter circle with 1ft radius counterclockwise, STARTING WHILE STATAIONARY AND ENDING AT MAX SPEED
arcRight(45, 1, 100, 2) // traces a 45 degree arc with 1ft radius clockwise, MAINTAINING SPEED AT THE BEGINNING AND END OF THE MOVE
arcLeft(135, 1, 100, 3) //traces a 135 degree arc with 1ft radius counterclockwise, COASTING TO A STOP AT THE END
```
In total, the 4 types of arc turn are:  
0 - Start and end stationary  
1 - Start stationary, end in motion  
2 - Start and end in motion  
3 - Start in motion, end stationary
___
##**sLeft/sRight**
```cpp
void sLeft(int arc1, int mid, int arc2, int max = 100);
void sRight(int arc1, int mid, int arc2, int max = 100);
```
Perform a forward S-shaped movement with a set length and speed.  
`_sLeft()` and `_sRight()` take the same arguments and perform the same move backwards.
___
##**tank**
```cpp
void tank(double left, double right);
```
Assign a direct motor voltage to each half of the drive. Passing joystick values into this function allows for tank driver operator controls.
___
##**arcade**
```cpp
void arcade(double vertical, double horizontal);
```
Assign a motor voltage to each half of the drive calculated as vertical +/- the horizontal. Passing joystick values into this function allows for arcade driver operator controls.
___
##**holonomic**
```cpp
void holonomic(double x, double y, double z);
```
Assign a motor voltage to each chassis motor individually based on a holonomic calculation. Passing joystick values into this function allows for holonomic driver operator controls.
___

# Odometry Functions

##**reset**
```cpp
void reset(std::array<double, 2> point = {0, 0});
```
Reset the current odometry position to the target coordinates.
___
##**reset**
```cpp
void reset(std::array<double, 2> point, double angle);
```
Reset both odometry position and angle.
___
##**getAngleError**
```cpp
double getAngleError(std::array<double, 2> point);
```
Returns the difference in robot robot heading from the target point in degrees.
___
##**getDistanceError**
```cpp
double getDistanceError(std::array<double, 2> point);
```
Returns the distance from the robot's current position to the target point.
___
##**moveAsync**
```cpp
void moveAsync(std::array<double, 2> point, double max = 80);
```
Moves the robot asynchronously towards the target point coordinates at the given maximum speed.
___
##**holoAsync**
```cpp
void holoAsync(std::array<double, 2> point, double angle, double max = 80, double turnMax = 50);
```
Moves the robot asynchronously towards the target point coordinates holonomically at the given maximum speed and maximum rate of turning.
___
##**move**
```cpp
void move(std::array<double, 2> point, double max = 80);
```
Moves the robot towards the target point coordinates at the given maximum speed.
___
##**moveThru**
```cpp
void moveThru(std::array<double, 2> point, double max = 80);
```
Moves towards a target point, but exits as soon as the robot enters a radius within the target point. See [EXIT_ERROR](#constants)
___
##**holo**
```cpp
void holo(std::array<double, 2> point, double angle, double max = 80, double turnMax = 50);
```
Moves the robot towards the target point coordinates holonomically at the given maximum speed and maximum rate of turning.
___
##**holoThru**
```cpp
void holoThru(std::array<double, 2> point, double angle, double max = 80, double turnMax = 50);
```
Moves towards a target point holonomically, but exits as soon as the robot enters a radius within the target point. See [EXIT_ERROR](#constants)
___

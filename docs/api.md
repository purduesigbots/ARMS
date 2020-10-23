# ARMS API

## Constants

### Chassis
| Constant | Default Value | Description |
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
**function_name**
```cpp
function_name(var1, var2);
```
`var1` - The first variable.

`var2` - The second variable.

A description of what the function does, including an example if necessary.
```cpp
example code
```

### Selection

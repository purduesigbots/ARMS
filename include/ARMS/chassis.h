#ifndef _ARMS_CHASSIS_H_
#define _ARMS_CHASSIS_H_

#include "ARMS/config.h"
#include "ARMS/pid.h"
#include "okapi/api.hpp"

namespace arms::chassis {

class Chassis {
protected:
	bool useVelocity;
	double accel_step;
	double arc_step;
	double distance_constant;
	double degree_constant;
	double width;
	double maxSpeed;
	double maxTurn;
	double maxAngular;
	double prev;
	double output_prev[4];

	double global_x;
	double global_y;
	double heading;
	double heading_degrees;

	std::shared_ptr<okapi::Motor> frontLeft;
	std::shared_ptr<okapi::Motor> frontRight;
	std::shared_ptr<okapi::Motor> backLeft;
	std::shared_ptr<okapi::Motor> backRight;

	std::shared_ptr<okapi::MotorGroup> leftMotors;
	std::shared_ptr<okapi::MotorGroup> rightMotors;

	std::shared_ptr<pros::ADIEncoder> leftEncoder;
	std::shared_ptr<pros::ADIEncoder> rightEncoder;
	std::shared_ptr<pros::ADIEncoder> middleEncoder;
	std::shared_ptr<pros::Imu> imu;

	int settle_count;
	int settle_time;
	double settle_threshold_linear;
	double settle_threshold_angular;

	int joystick_threshold;

	pid::PID pid;

public:
	/// Gets the values of the left and right encoders.
	std::array<double, 2> getEncoders();

	/// Checks if a wheel is moving
	int wheelMoving(double sv, double* psv);

	/// Set the speed of target motor
	void motorMove(std::shared_ptr<okapi::Motor> motor, double speed);

	/// Set the speed of target motor group
	void motorMove(std::shared_ptr<okapi::MotorGroup> motor, double speed);

	/// Set the speed of target motor
	void motorMove(std::shared_ptr<okapi::Motor> motor, double speed, bool vel);

	/// Set the speed of target motor group
	void motorMove(std::shared_ptr<okapi::MotorGroup> motor, double speed,
	               bool vel);

	/// Set the brake mode for all chassis motors
	void setBrakeMode(okapi::AbstractMotor::brakeMode b);

	/// Reset imu if it is being used
	void resetAngle(double angle = 0);

	/// Reset the internal motor encoders for all chassis motors
	void reset();

	/// Get the average position between the sides of the chassis
	double position(bool yDirection = false);

	/// Get the angle of the chassis in degrees
	double angle();

	/// Get the difference between the sides of the chassis
	double difference();

	/// Reduce a speed
	double limitSpeed(double speed, double max);

	/// Get a gradually accelerating speed towards the target input
	double slew(double speed, double step, double* prev);

	/// Get a boolean that is true if the chassis motors are in motion
	bool settled();

	/// Delay the program until the chassis motors come to rest
	void waitUntilSettled();

	/// Begin an asycronous chassis movement
	void moveAsync(double sp, int max = 100);

	/// Begin an asycronous turn movement
	void turnAsync(double sp, int max = 100);

	/// Begin an asycronous absolute turn movement (only works with IMU)
	void turnAbsoluteAsync(double sp, int max = 100);

	/// Begin an asycronous holonomic chassis movement
	void holoAsync(double distance, double angle, int max = 100);

	/// Perform a chassis movement and wait until settled
	void move(double sp, int max = 100);

	/// Perform a turn movement and wait until settled
	void turn(double sp, int max = 100);

	/// Perform an absolute turn movement using IMU and wait until settled
	void turnAbsolute(double sp, int max = 100);

	/// Perform a holonomic movement and wait until settled
	void holo(double distance, double angle, int max = 100);

	/// Move a distance at a set voltage with no PID
	void fast(double sp, int max = 100);

	/// Move for a duration at a set voltage with no PID
	void voltage(int t, int left_speed = 100, int right_speed = 101);

	/// Move for a duration at a set velocity using internal PID
	void velocity(int t, int left_max = 100, int right_max = 101);

	/// Move in an arc of given length and radius
	void arc(bool mirror, int arc_length, double rad, int max, int type);

	/// Move the robot in an arc with a set length, radius, and speed
	void arcLeft(int length, double rad, int max = 100, int type = 0);

	/// Move the robot in an arc with a set length, radius, and speed
	void arcRight(int length, double rad, int max = 100, int type = 0);

	// Move in an S-curve
	void scurve(bool mirror, int arc1, int mid, int arc2, int max);

	/// Perform a forward S shaped movement with a set length, and speed
	void sLeft(int arc1, int mid, int arc2, int max = 100);

	/// Perform a forward S shaped movement with a set length, and speed
	void sRight(int arc1, int mid, int arc2, int max = 100);

	/// Perform a backward S shaped movement with a set length, and speed
	void _sLeft(int arc1, int mid, int arc2, int max = 100);

	/// Perform a backward S shaped movement with a set length, and speed
	void _sRight(int arc1, int mid, int arc2, int max = 100);

	/// Assign a voltage to each motor on a scale of -100 to 100
	void tank(double left, double right);

	/// Assign a voltage to each motor on a scale of -100 to 100
	void arcade(double vertical, double horizontal);

	/// Assign a voltage to each motor on a scale of -100 to 100
	void holonomic(double x, double y, double z);

	/// Start the asynchronous task for this chassis
	void startTask();

	/// A class to control a robot chassis
	Chassis(std::initializer_list<okapi::Motor> leftMotorsList = {LEFT_MOTORS},
	        std::initializer_list<okapi::Motor> rightMotorsList = {RIGHT_MOTORS},
	        int gearset = GEARSET, double distance_constant = DISTANCE_CONSTANT,
	        double degree_constant = DEGREE_CONSTANT,
	        int settle_time = SETTLE_TIME,
	        double settle_threshold_linear = SETTLE_THRESHOLD_LINEAR,
	        double settle_threshold_angular = SETTLE_THRESHOLD_ANGULAR,
	        double accel_step = ACCEL_STEP, double arc_step = ARC_STEP,
	        int imuPort = IMU_PORT,
	        std::tuple<int, int, int> encoderPorts = {ENCODER_PORTS},
	        int expanderPort = EXPANDER_PORT,
	        int joystick_threshold = JOYSTICK_THRESHOLD,
	        pid::PID pid = pid::PID());
};

class OdomChassis : public chassis::Chassis {
protected:
	bool debug;
	double left_right_distance;
	double middle_distance;
	double left_right_tpi;
	double middle_tpi;
	double exit_error;
	bool holonomic;

	double prev_heading;
	double prev_right_pos;
	double prev_left_pos;
	double prev_middle_pos;

public:
	/// Resets odometry to the given point
	void resetOdom(std::array<double, 2> point = {0, 0});

	/// Resets odometry to the given point and heading
	void resetOdom(std::array<double, 2> point, double angle);

	/// Gets the error between current chassis heading and the target heading
	double getAngleError(std::array<double, 2> point);

	/// Gets the distance between current chassis position and the target position
	double getDistanceError(std::array<double, 2> point);

	/// Begin an asynchronous chassis movement with odometry
	void moveAsync(std::array<double, 2> point, double max = 80);

	/// Begin an asynchronous holonomic chassis movement with odometry
	void holoAsync(std::array<double, 2> point, double angle, double max = 80,
	               double turnMax = 50);

	/// Begin a blocking chassis movement with odometry
	void move(std::array<double, 2> point, double max = 80);

	/// Begin a blocking chassis movement with odometry without end PID
	void moveThru(std::array<double, 2> point, double max = 80);

	/// Begin a blocking holonomic chassis movement with odometry
	void holo(std::array<double, 2> point, double angle, double max = 80,
	          double turnMax = 50);

	/// Begin a blocking holonomic chassis movement with odometry without end PID
	void holoThru(std::array<double, 2> point, double angle, double max = 80,
	              double turnMax = 50);

	/// Find the intersection points a path and a circle with given radius
	std::array<double, 2>
	findIntersectionPoint(std::vector<std::array<double, 2>> path, double radius);

	/// Follow a path using odometry
	void followPath(std::vector<std::array<double, 2>> path);

	/// Start the asynchronous task for this odometry chassis
	void startTask();

	/// A class to control a robot chassis with odometry
	OdomChassis(
	    std::initializer_list<okapi::Motor> leftMotorsList = {LEFT_MOTORS},
	    std::initializer_list<okapi::Motor> rightMotorsList = {RIGHT_MOTORS},
	    int gearset = GEARSET, double distance_constant = DISTANCE_CONSTANT,
	    double degree_constant = DEGREE_CONSTANT, int settle_time = SETTLE_TIME,
	    double settle_threshold_linear = SETTLE_THRESHOLD_LINEAR,
	    double settle_threshold_angular = SETTLE_THRESHOLD_ANGULAR,
	    double accel_step = ACCEL_STEP, double arc_step = ARC_STEP,
	    int imuPort = IMU_PORT,
	    std::tuple<int, int, int> encoderPorts = {ENCODER_PORTS},
	    int expanderPort = EXPANDER_PORT,
	    int joystick_threshold = JOYSTICK_THRESHOLD, pid::PID pid = pid::PID(),
	    bool debug = ODOM_DEBUG, double left_right_distance = LEFT_RIGHT_DISTANCE,
	    double middle_distance = MIDDLE_DISTANCE,
	    double left_right_tpi = LEFT_RIGHT_TPI, double middle_tpi = MIDDLE_TPI,
	    bool holonomic = HOLONOMIC, double exit_error = EXIT_ERROR);
};

class Odom {
protected:
	bool debug;
	double left_right_distance;
	double middle_distance;
	double left_right_tpi;
	double middle_tpi;
	double exit_error;
	bool holonomic;

public:
	/// Sets the debug value
	Odom& withDebug(bool deb);

	/// Sets the distance values
	Odom& withDistances(double left_right, double middle);

	/// Sets the TPI values
	Odom& withTPI(double left_right, double middle);

	/// Sets the exit error value
	Odom& withExitError(double err);

	/// Sets the holonomic value
	Odom& withHolonomic(bool holo);

	/// Gets the debug value
	bool getDebug();

	/// Gets the left/right distance value
	double getLeftRightDistance();

	/// Gets the middle distance value
	double getMiddleDistance();

	/// Gets the left/right TPI value
	double getLeftRightTPI();

	/// Gets the middle TPI value
	double getMiddleTPI();

	/// Gets the exit error value
	double getExitError();

	/// Gets the holonomic value
	bool getHolonomic();

	/// A class for holding odometry values
	Odom();
};

class ChassisBuilder {
protected:
	std::initializer_list<okapi::Motor> leftMotorsList;
	std::initializer_list<okapi::Motor> rightMotorsList;
	int gearset;
	double distance_constant;
	double degree_constant;
	int settle_time;
	double settle_threshold_linear;
	double settle_threshold_angular;
	double accel_step;
	double arc_step;
	int imuPort;
	std::tuple<int, int, int> encoderPorts;
	int expanderPort;
	int joystick_threshold;
	pid::PID pid;

	bool has_odom;
	Odom odom;

public:
	/// Sets the motors
	ChassisBuilder& withMotors(std::initializer_list<okapi::Motor> leftMotors,
	                           std::initializer_list<okapi::Motor> rightMotors);

	/// Sets the gearset
	ChassisBuilder& withGearset(int gs);

	/// Sets the distance constant
	ChassisBuilder& withDistanceConstant(double dist);

	/// Sets the degree constant
	ChassisBuilder& withDegreeConstant(double deg);

	/// Sets the settle time
	ChassisBuilder& withSettleTime(int time);

	/// Sets the linear settle threshold
	ChassisBuilder& withLinearSettleThreshold(double threshold);

	/// Sets the angular settle threshold
	ChassisBuilder& withAngularSettleThreshold(double threshold);

	/// Sets the acceleration slew step
	ChassisBuilder& withAccelStep(double accel);

	/// Sets the arc slew step
	ChassisBuilder& withArcStep(double arc);

	/// Sets the IMU port
	ChassisBuilder& withIMU(int port);

	/// Sets the encoder ports
	ChassisBuilder& withEncoders(std::tuple<int, int, int> ports);

	/// Sets the expander port
	ChassisBuilder& withExpander(int port);

	/// Sets the joystick threshold
	ChassisBuilder& withJoystickThreshold(int threshold);

	/// Sets the PID values
	ChassisBuilder& withPID(pid::PID p);

	/// Sets the Odom values
	ChassisBuilder& withOdom(Odom o);

	/// Builds a Chassis object
	Chassis build();

	/// Builds an OdomChassis object
	OdomChassis buildOdom();

	/// A builder for creating Chassis and OdomChassis objects
	ChassisBuilder();
};

} // namespace arms::chassis

#endif

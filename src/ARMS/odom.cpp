#include "ARMS/lib.h"
#include "api.h"

namespace arms::odom {

config_data_s_t configData;

// sensors
std::shared_ptr<pros::Imu> imu = nullptr;
std::shared_ptr<pros::Rotation> rightRotation = nullptr;
std::shared_ptr<pros::Rotation> leftRotation = nullptr;
std::shared_ptr<pros::Rotation> middleRotation = nullptr;
std::shared_ptr<pros::ADIEncoder> rightADIEncoder = nullptr;
std::shared_ptr<pros::ADIEncoder> leftADIEncoder = nullptr;
std::shared_ptr<pros::ADIEncoder> middleADIEncoder = nullptr;

// output the odometry data to the terminal
bool debug;

// tracker wheel configuration
double track_width;
double left_right_distance;
double middle_distance;
double tpi;
double middle_tpi;

// odom position values
Point position;
double heading;

// previous values
double prev_left_pos = 0;
double prev_right_pos = 0;
double prev_middle_pos = 0;
double prev_heading = 0;

double getLeftEncoder() {
	if (configData.encoderType == ENCODER_ADI && leftADIEncoder != nullptr) {
		return leftADIEncoder->get_value();
	} else if (leftRotation != nullptr) {
		return leftRotation->get_position();
	} else if (chassis::leftMotors) {
		return chassis::leftMotors->get_positions()[0];
	}
	return 0;
}

double getRightEncoder() {
	if (configData.encoderType == ENCODER_ADI && rightADIEncoder != nullptr) {
		return rightADIEncoder->get_value();
	} else if (rightRotation != nullptr) {
		return rightRotation->get_position();
	} else if (chassis::rightMotors) {
		return chassis::rightMotors->get_positions()[0];
	}
	return 0;
}

double getMiddleEncoder() {
	if (configData.encoderType == ENCODER_ADI && middleADIEncoder != nullptr) {
		return middleADIEncoder->get_value();
	} else if (middleRotation != nullptr) {
		return middleRotation->get_position();
	}
	return 0;
}

int odomTask() {

	position.x = 0;
	position.y = 0;
	heading = 0;

	while (true) {
		// get positions of each encoder
		double left_pos = getLeftEncoder();
		double right_pos = getRightEncoder();
		double middle_pos = configData.middleEncoderPort ? getMiddleEncoder() : 0;

		// calculate change in each encoder
		double delta_left = (left_pos - prev_left_pos) / tpi;
		double delta_right = (right_pos - prev_right_pos) / tpi;
		double delta_middle = configData.middleEncoderPort
		                          ? (middle_pos - prev_middle_pos) / middle_tpi
		                          : 0;

		// calculate new heading
		double delta_angle;
		if (imu) {
			heading = -imu->get_rotation() * M_PI / 180.0;
			delta_angle = heading - prev_heading;
		} else {
			delta_angle = (delta_right - delta_left) / track_width;

			heading += delta_angle;
		}

		// store previous positions
		prev_left_pos = left_pos;
		prev_right_pos = right_pos;
		prev_middle_pos = middle_pos;
		prev_heading = heading;

		// calculate local displacement
		double local_x;
		double local_y;

		if (delta_angle) {
			double i = sin(delta_angle / 2.0) * 2.0;
			local_x = (delta_right / delta_angle - left_right_distance) * i;
			local_y = (delta_middle / delta_angle + middle_distance) * i;
		} else {
			local_x = delta_right;
			local_y = delta_middle;
		}

		double p = heading - delta_angle / 2.0; // global angle

		// convert to absolute displacement
		position.x += cos(p) * local_x - sin(p) * local_y;
		position.y += cos(p) * local_y + sin(p) * local_x;

		if (debug)
			printf("%.2f, %.2f, %.2f \n", position.x, position.y, getHeading());

		pros::delay(10);
	}
}

void reset(Point point) {
	position.x = point.x;
	position.y = point.y;
}

void reset(Point point, double angle) {
	reset(point);
	heading = angle * M_PI / 180.0;
	prev_heading = heading;
	if (imu)
		imu->set_rotation(-angle);
}

Point getPosition() {
	return position;
}

double getHeading(bool radians) {
	if (radians)
		return heading;
	return heading * 180 / M_PI;
}

double getAngleError(Point point) {
	double x = point.x;
	double y = point.y;

	x -= position.x;
	y -= position.y;

	double delta_theta = atan2(y, x) - heading;

	// if movement is reversed, calculate delta_theta using a 180 degree rotation
	// of the target point
	if (pid::reverse) {
		delta_theta = atan2(-y, -x) - heading;
	}

	while (fabs(delta_theta) > M_PI) {
		delta_theta -= 2 * M_PI * delta_theta / fabs(delta_theta);
	}

	return delta_theta;
}

double getDistanceError(Point point) {
	double x = point.x;
	double y = point.y;

	y -= position.y;
	x -= position.x;
	return sqrt(x * x + y * y);
}

void init(bool debug, EncoderType_e_t encoderType,
          std::array<int, 3> encoderPorts, int expanderPort, int imuPort,
          double track_width, double middle_distance, double tpi,
          double middle_tpi) {
	odom::debug = debug;
	odom::track_width = track_width;
	odom::left_right_distance = track_width / 2;
	odom::middle_distance = middle_distance;
	odom::tpi = tpi;
	odom::middle_tpi = middle_tpi;

	pros::Task odom_task(odomTask);

	configData.expanderPort = expanderPort;
	configData.leftEncoderPort = encoderPorts[0];
	configData.rightEncoderPort = encoderPorts[1];
	configData.middleEncoderPort = encoderPorts[2];
	configData.encoderType = encoderType;

	// Initialize devices
	switch (encoderType) {
	case ENCODER_ADI:
		if (expanderPort == 0) {
			if (configData.leftEncoderPort != 0) {
				leftADIEncoder = std::make_shared<pros::ADIEncoder>(
				    abs(configData.leftEncoderPort),
				    abs(configData.leftEncoderPort) + 1,
				    configData.leftEncoderPort < 0);
				rightADIEncoder = std::make_shared<pros::ADIEncoder>(
				    abs(configData.rightEncoderPort),
				    abs(configData.rightEncoderPort) + 1,
				    configData.rightEncoderPort < 0);
			}
			if (configData.middleEncoderPort != 0) {
				middleADIEncoder = std::make_shared<pros::ADIEncoder>(
				    abs(configData.middleEncoderPort),
				    abs(configData.middleEncoderPort) + 1,
				    configData.middleEncoderPort < 0);
			}
		} else {
			if (configData.leftEncoderPort != 0) {
				leftADIEncoder = std::make_shared<pros::ADIEncoder>(
				    std::tuple<int, int, int>({expanderPort,
				                               abs(configData.leftEncoderPort),
				                               abs(configData.leftEncoderPort) + 1}),
				    configData.leftEncoderPort < 0);
				rightADIEncoder = std::make_shared<pros::ADIEncoder>(
				    std::tuple<int, int, int>({expanderPort,
				                               abs(configData.rightEncoderPort),
				                               abs(configData.rightEncoderPort) + 1}),
				    configData.rightEncoderPort < 0);
			}
			if (configData.middleEncoderPort != 0) {
				middleADIEncoder = std::make_shared<pros::ADIEncoder>(
				    std::tuple<int, int, int>({expanderPort,
				                               abs(configData.middleEncoderPort),
				                               abs(configData.middleEncoderPort) + 1}),
				    configData.middleEncoderPort < 0);
			}
		}
		break;
	case ENCODER_ROTATION:
		if (configData.leftEncoderPort != 0) {
			leftRotation = std::make_shared<pros::Rotation>(
			    configData.leftEncoderPort, configData.leftEncoderPort < 0);
			rightRotation = std::make_shared<pros::Rotation>(
			    configData.rightEncoderPort, configData.rightEncoderPort < 0);
		}
		if (configData.middleEncoderPort != 0)
			middleRotation = std::make_shared<pros::Rotation>(
			    configData.middleEncoderPort, configData.middleEncoderPort < 0);
		break;
	default:
		break;
	}
	// initialize imu
	if (imuPort != 0) {
		imu = std::make_shared<pros::Imu>(imuPort);
		imu->reset(true);
	}
	pros::delay(100);
	reset();
}

} // namespace arms::odom

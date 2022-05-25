#ifndef _ARMS_POSE_H_
#define _ARMS_POSE_H_

namespace arms {

/**
 * This is a helper union that defines a 2d point with an angle.
 *
 * A union was chosen to represent the point so that a couple different names
 * can be used to access the same elements. For example:
 *      pose.x, pose.data[0], and pose[0]
 * refer to the same variable.
 * The point part of the point angle can be acessed as point.p
 */
union Pose {
	double& operator[](unsigned int index) {
		return data[index];
	}

	struct {
		double x, y, a;
	};
	Point p;
	double data[3];
};

} // namespace arms

#endif //_ARMS_POSE_H_
#include "ARMS/api.h"
#include "api.h"

#include <cmath>
#include <tuple>

using namespace pros;

namespace arms::purepursuit {

// Beware all ye who feast their eyes upon this source
// This code from matlab to c++, we have forced
// The working of this algorithm none shall know

int projectionLineIndex = 0;
Point projectionPoint = {0, 0};
double lookahead;

/**
 * Find total length of path from robot
 */
double getDistanceError(std::vector<Point> waypoints) {
	double dist = length(waypoints[projectionLineIndex + 1] - projectionPoint);

	for (int i = projectionLineIndex + 1; i <= waypoints.size() - 2; i++) {
		dist += length(waypoints[i + 1] - waypoints[i]);
	}

	return dist;
}

/**
 * Find the closest point on a line
 */
std::tuple<Point, double> closestPointOnLine(Point pt1, Point pt2,
                                             Point refPt) {

	Point closestPoint;

	// Vector from pt1 to pt2
	Point v12 = pt2 - pt1;
	// Vector from refPt to pt2
	Point vr2 = pt2 - refPt;

	// Projection of the vr2 on v12, normalized by norm(v12)
	double alpha =
	    (v12.x * vr2.x + v12.y * vr2.y) / (v12.x * v12.x + v12.y * v12.y);

	// Find the closet point by interpolation
	if (alpha > 1 || isnan(alpha))
		closestPoint = pt1;
	else if (alpha < 0)
		closestPoint = pt2;
	else
		closestPoint = {alpha * pt1.x + (1 - alpha) * pt2.x,
		                alpha * pt1.y + (1 - alpha) * pt2.y};

	double distance = length(refPt - closestPoint);
	return {closestPoint, distance};
}

/**
 * Compute projection point
 */
void computeProjectionPoint(std::vector<Point> waypoints) {
	// computeprojectionPoint Find closest point past the last Projection point
	bool searchFlag = false;
	Point robotPosition{odom::global_x, odom::global_y};

	// If Projection point is not initialized, start searching from
	// first waypoint
	if (projectionLineIndex == 0) {
		searchFlag = true;
		projectionPoint = waypoints[0];
		projectionLineIndex = 0;
	}

	// Start searching from the current projection line segment
	std::tuple<Point, int> pointData = closestPointOnLine(
	    projectionPoint, waypoints[projectionLineIndex + 1], robotPosition);
	projectionPoint = std::get<0>(pointData);
	double minDistance = std::get<1>(pointData);

	double dist = length(projectionPoint - waypoints[projectionLineIndex + 1]);

	for (int i = projectionLineIndex + 1; i < waypoints.size() - 1; i++) {
		if (!searchFlag && dist > lookahead) {
			break;
		}

		dist = dist + length(waypoints[i] - waypoints[i + 1]);

		// Check the remaining waypoints
		std::tuple tempClosestPointData =
		    closestPointOnLine(waypoints[i], waypoints[i + 1], robotPosition);

		Point tempPoint = std::get<0>(tempClosestPointData);
		double tempDistance = std::get<1>(tempClosestPointData);

		if (tempDistance < minDistance) {
			minDistance = tempDistance;
			projectionPoint = tempPoint;
			projectionLineIndex = i;
		}
	}
}

/**
 *  Find the lookahead point for a given set of waypoints
 */
Point getLookaheadPoint(std::vector<Point> waypoints) {

	computeProjectionPoint(waypoints);

	// First check the current line segment
	double dist = length(projectionPoint - waypoints[projectionLineIndex + 1]);
	Point lookaheadStartPt = projectionPoint;
	Point lookaheadEndPt = waypoints[projectionLineIndex + 1];
	double overshootDist = dist - lookahead;
	int lookaheadIdx = projectionLineIndex;

	// If the remaining path on current line segment is not long
	// enough for look ahead, check the waypoints past current line
	// segment.
	while (overshootDist < 0) {
		if (lookaheadIdx >= waypoints.size() - 1)
			return lookaheadEndPt;

		lookaheadIdx++;

		lookaheadStartPt = waypoints[lookaheadIdx];
		lookaheadEndPt = waypoints[lookaheadIdx + 1];
		dist += length(lookaheadStartPt - lookaheadEndPt);

		overshootDist = dist - lookahead;
	}

	// Find the exact look ahead point by interpolating between the
	// start and end point of the line segment on which the
	// lookahead point lies.

	// prevent division by 0
	if (length(lookaheadStartPt - lookaheadEndPt) == 0)
		return lookaheadEndPt;

	Point lookaheadPoint;
	double alpha = overshootDist / length(lookaheadStartPt - lookaheadEndPt);
	lookaheadPoint = {alpha * lookaheadStartPt.x + (1 - alpha) * lookaheadEndPt.x,
	                  alpha * lookaheadStartPt.y +
	                      (1 - alpha) * lookaheadEndPt.y};

	return lookaheadPoint;
}

/**
 * Initialize tuning paramters for purepursuit
 */
void init(double lookahead) {
	purepursuit::lookahead = lookahead;
}

} // namespace arms::purepursuit

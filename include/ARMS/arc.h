#ifndef _ARC_H_
#define _ARC_H_

namespace chassis {

/**
 * Move the robot in an arc with a set length, radius, and speed
 */
void arcLeft(int length, double rad, int max = 100, int type = 0);

/**
 * Move the robot in an arc with a set length, radius, and speed
 */
void arcRight(int length, double rad, int max = 100, int type = 0);

/**
 * Preform a forward S shaped movement with a set length, and speed
 */
void sLeft(int arc1, int mid, int arc2, int max = 100);

/**
 * Preform a forward S shaped movement with a set length, and speed
 */
void sRight(int arc1, int mid, int arc2, int max = 100);

/**
 * Preform a backward S shaped movement with a set length, and speed
 */
void _sLeft(int arc1, int mid, int arc2, int max = 100);

/**
 * Preform a backward S shaped movement with a set length, and speed
 */
void _sRight(int arc1, int mid, int arc2, int max = 100);

} // namespace chassis

#endif

#ifndef CONVEX_H
#define CONVEX_H

#include "../include/project1/wallFollower.h"

/**
 * 
 * CDriveForward uses a PI controller to maintain a distance from the wall.
 * It calculates the error by taking the difference between the LiDAR 
 * distance measurement and the threshold distance it wants to be at.
 * It calculates the measured distance by taking the minimum distance to
 * the left of the robot within a range, then fanning outwards and averaging
 * the cosines of the adjascent measurements to get a more robust averaged
 * measurement. The PI then adjusts its course to move closer or further
 * away from the wall.
 * 
 * It interfaces with CWallFollower to obtain the LiDAR data, and to compute
 * the moving average.
 * 
*/
class convexState {
public:
    void turnLeft(CWallFollower* bot);

private:
    double radius_ = 0.1; //150mm
};

#endif // CONVEX_H

#ifndef CONVEX_H
#define CONVEX_H

#include "../include/project1/wallFollower.h"

class convexState {
public:
    void turnLeft(CWallFollower* bot);

private:
    double radius_ = 0.15; //150mm
};

#endif // CONVEX_H

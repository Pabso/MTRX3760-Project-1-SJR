#include "../include/project1/convexState.h"
#include<cmath>

void convexState::turnLeft(CWallFollower* bot) {

    // calculate the angular velocity to achieve the 150mm radius arc
    double linearVelocity = bot->linearV; 
    double angularVelocity = linearVelocity / radius_;

    bot->linearV = linearVelocity;
    bot->angularV = angularVelocity;

    return;
}

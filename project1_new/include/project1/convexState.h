#ifndef CONVEX_H
#define CONVEX_H

#include "../include/project1/wallFollower.h"

class convexState {
public:
    //convexState(ros::NodeHandle& nh);
    void turnLeft(CWallFollower* bot);

private:
    //ros::NodeHandle nh_;
    //ros::Subscriber laser_sub_;
    double dist;
    double threshold_ = 150; //150mm
    bool turning_ = false;
};

#endif // CONVEX_H

#include "../include/project1/convexState.h"


void laserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {   
    
    //get distance at angle 0 - left 
    dist = msg->ranges.at(0)
}

void convexState::init() {
    ros::Subscriber sub_laser = n.subscribe("/scan",1,laserScan);
}

void convexState::turnLeft(CWallFollower* bot) {

    // initalise lidar
    init();

    if (!turning_) {
        
        // if not already, start turning left
        updatecommandVelocity(1.0, 1.0); // linear, angular vel
        turning_ = true;
             
        // check for wall within the threshold at left point
        if (dist < threshold_) {
                
            // when detected, stop turning
            updatecommandVelocity(0, 0);

            // exit the state 
            turning_ = false;
            bot->States nextState = DRIVE_FOWARD;
        }
    }
}

#include "../include/project1/convexState.h"

convexState::convexState() {
    
    // scan data to detect obstacles
    laser_sub_ = nh_.subscribe("/scan", 1, &convexState::stopTurn, this);

    // send Twist commands
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void convexState::turnLeft() {
    if (!turning_) {
        
        // if not already, start turning left
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = 1.0;  // angular velocity
        cmd_vel.linear.x = 0.2;   // linear velocity 
        cmd_vel_pub_.publish(cmd_vel);
        turning_ = true;
    }
}

void convexState::stopTurn(const sensor_msgs::LaserScan::ConstPtr& scan) {
    
    // check for wall within the threshold
    for (int i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < threshold_) {
            
            // when detected, stop turning and face forward
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 0.0;    // angular velocity
            cmd_vel.linear.x = 0.0;     // linear velocity
            cmd_vel_pub_.publish(cmd_vel);

            // exit the state 
            turning_ = false;
            return;
        }
    }
}

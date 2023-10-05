#include "../include/project1/convexState.h"

convexState::convexState() {
    
    // scan data to detect obstacles
    laser_sub_ = nh_.subscribe("/scan", 1, &convexState::turnLeft, this);

    // send Twist commands
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void convexState::turnLeft(const sensor_msgs::LaserScan::ConstPtr& scan) {
    if (!turning_) {
        
        // if not already, start turning left
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = 1.0;  // angular velocity
        cmd_vel.linear.x = 0.2;   // linear velocity 
        cmd_vel_pub_.publish(cmd_vel);
        turning_ = true;

        // get number of points
        int pts = scan->ranges.size();

        // make sure pts > 0
        if (pts > 0) {
            
            // initlialise left pt
            int leftIndex = 0; 
    
            // check for wall within the threshold at left point
            if (scan->ranges[leftIndex] < threshold_) {
                
                // when detected, stop turning
                cmd_vel.angular.z = 0.0;    // angular velocity
                cmd_vel.linear.x = 0.0;     // linear velocity
                cmd_vel_pub_.publish(cmd_vel);

                // exit the state 
                turning_ = false;
                return;
            }
        }
    }
}

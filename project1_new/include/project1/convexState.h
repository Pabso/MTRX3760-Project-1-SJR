#ifndef CONVEX_H
#define CONVEX_H

#include "../include/project1/wallFollower.h"

//#include </opt/ros/noetic/share/ros/ros.h>                already in wallFollower??
//#include </opt/ros/noetic/share/geometry_msgs/Twist.h>    already in wallFollower??
//#include </opt/ros/noetic/share/sensor_msgs/LaserScan.h>  already in wallFollower??

class convexState {
public:
    convexState(ros::NodeHandle& nh);
    void turnLeft(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;
    double threshold_ = 0.15; //150mm
    bool turning_ = false;
};

#endif // CONVEX_H

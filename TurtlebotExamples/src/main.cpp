// Minimal example for getting a new turtlebot project started
// Not good coding style, this is intended to get you started

// Original written by Tara Bartlett, 2019
// Updated 2020 Donald Dansereau

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// declare a publisher and subscriber
// note this example uses global variables and functions, these should really be encapsulated,
// e.g. by building all of this into a class
ros::Publisher pub_vel_;
ros::Subscriber sub_laser_;

void PublishVelocity( const std::vector<float> lin_vel, const std::vector<float> ang_vel )
{
    geometry_msgs::Twist msg;
    
    msg.linear.x = lin_vel[0];
    msg.linear.y = lin_vel[1];
    msg.linear.z = lin_vel[2];

    msg.angular.x = ang_vel[0];
    msg.angular.y = ang_vel[1];
    msg.angular.z = ang_vel[2];
    
    pub_vel_.publish(msg);
}

void LaserCallback( const sensor_msgs::LaserScan& msg )
{
    //message members: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html 
    
    //example for how to access members
    float min_angle = msg.angle_min;
}

int main( int argc, char** argv )
{
    // create a ROS node
    ros::init( argc, argv, "SampleTurtlebotNode" );
    ros::NodeHandle nh;
    ros::NodeHandle pn( "~" );

    // initialise publisher and subscriber
    pub_vel_   = nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 10 );
    sub_laser_ = nh.subscribe( "/scan", 1000, LaserCallback );

    ros::Rate loop_rate( 30 );
    while( ros::ok() )
    { 
        // initialise 0-vectors
        std::vector<float> lin_vel( 3, 0 );
        std::vector<float> ang_vel( 3, 0 );
        lin_vel[0] = 1; // go forwards

        // publish the velocity
        PublishVelocity( lin_vel, ang_vel );

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

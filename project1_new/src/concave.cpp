#include "../include/project1/ConcaveCorner.h"

void CConcaveCorner::clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    int increment = 0;
    double dist;
    double real_dist;
    for (int num = MIN_BOUNDRY_ANGLE; num < MAX_BOUNDRY_ANGLE; num++)
    {
        //Account for offset of lidar
        dist = msg->ranges.at(num)
        real_dist = sqrt(pow((dist*sin(num)+TURTLEBOT_LIDAR_OFFSET_Y),2) + pow((dist*cos(num)+TURTLEBOT_LIDAR_OFFSET_X),2))

        ///If the scanned distance is inf set to range_max
    if ((real_dist < MAX_BOUNDRY_DIA) && incrment < 2)
        {
        lidar_Call[increment] = num;
        increment++;
        }
    }
}

void CConcaveCorner::init(CWallFollower* bot)
{
    //Set Boundry Size
    MAX_BOUNDRY_DIA = bot->bubble_size_;

    //Initalise the Laser to gather information 
    ros::Subscriber sub_laser = n.subscribe("scan",10, &CConcaveCorner::clbk_laser, this);
}

void CConcaveCorner::handler(CWallFollower* bot)
{
    //initalise the Lidar
    init(bot);

    //Average Angle
    double delta_angle = (lidar_Call[0] + lidar_Call[1])/2;

    //Activate Turn
    bot->turnOdom(delta_angle);

    //Set new State
    bot->States nextState = DRIVE_FOWARD;

    return;
} 
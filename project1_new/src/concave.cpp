#include "../include/project1/ConcaveCorner.h"
#include <cmath>

// void CConcaveCorner::clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
// {   
//     // int increment = 0;
//     // double dist;
//     // double real_dist;
//     // for (int num = MIN_BOUNDRY_ANGLE; num < MAX_BOUNDRY_ANGLE; num++)
//     // {
//     //     //Account for offset of lidar
//     //     dist = msg->ranges.at(num);
//     //     real_dist = sqrt(pow((dist*sin(num)+TURTLEBOT_LIDAR_OFFSET_Y),2) + pow((dist*cos(num)+TURTLEBOT_LIDAR_OFFSET_X),2));

//     //     ///If the scanned distance is inf set to range_max
//     // if ((real_dist < MAX_BOUNDRY_DIA) && increment < 2)
//     //     {
//     //     lidar_Call[increment] = num;
//     //     increment++;
//     //     }
//     // }
// }

void CConcaveCorner::init(CWallFollower* bot)
{
    //Gather Lidar Data
    laser_data_sraper(bot);
}

void CConcaveCorner::laser_data_sraper(CWallFollower* bot)
{   
    int increment = 0;

    for (const auto &pair : bot->mScanDataRange)
    {
        if(pair.second < 2*bot->bubble_size_ && pair.first > 20)
        { 
            laser_angle_sum = (pair.first/180)*M_PI;
            increment++;
        }
    }

    laser_angle_avg = laser_angle_sum/increment;
}

void CConcaveCorner::handler(CWallFollower* bot)
{
    //initalise the Lidar
    init(bot);

    //Activate Turn
    bot->turnOdom(laser_angle_avg);

    //Set new State
    bot->nextState = bot->States::DRIVE_FOWARD;

    return;
} 
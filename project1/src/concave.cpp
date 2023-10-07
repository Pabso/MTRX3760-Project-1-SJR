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
    laser_angle_sum = 0.0;

    for (const auto &pair : bot->filteredScanData)
    {
        if(pair.second < bot->bubble_size_*1.5 && pair.second > 1e-5 && pair.first >= 30 && pair.first <= 150)
        { 
            laser_angle_sum += pair.first;
            increment++;
        }
    }
    
    laser_angle_avg = laser_angle_sum/increment;
    laser_angle_avg = ((laser_angle_avg*M_PI)/180);
    //std::cout << "laser_angle_av "<< laser_angle_avg << std::endl;
}

void CConcaveCorner::handler(CWallFollower* bot)
{   
    static int counter = 0;
    if (bot->previousState == bot->States::DRIVE_FOWARD)
    {
        counter +=1;
    }
    //initalise the Lidar
    init(bot);
    if(counter > 0 && counter < 4)
    {
        counter +=1;
        bot->nextState = bot->States::CONCAVE_CORNER;
    }
    else {
        counter = 0;
        //Activate Turn
        std::cout << "angle: "<< laser_angle_avg << " sum:" << laser_angle_sum << std::endl;
        bot->turnOdom(laser_angle_avg-0.2);
        //Set new State
        bot->nextState = bot->States::DRIVE_FOWARD;
    }


    return;
} 
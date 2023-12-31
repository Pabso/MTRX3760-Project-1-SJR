#include "../include/project1/convexState.h"
#include<cmath>

void convexState::turnLeft(CWallFollower* bot) {

    double linearVelocity = 0.05; 
    double angularVelocity = 0.4;

    bot->linearV = linearVelocity;
    bot->angularV = angularVelocity;
    bool wall_detected = false;
    int sum = 0;
    for (const auto &pair : bot->filteredScanData)
    {
      if(pair.second < bot->bubble_size_ && pair.second > 1e-5 && pair.first >= 30 && pair.first <= 150)
        { 
          sum += 1;
        }
    }
    wall_detected = (sum > 3) ? true : false;

    if (wall_detected)
    {
      int increment = 0;
      int laser_angle_sum = 0.0;
      int laser_angle_avg = 0.0;

      for (const auto &pair : bot->filteredScanData)
      {
          if(pair.second < bot->bubble_size_*1.5 && pair.second > 1e-5 && pair.first >= 30 && pair.first <= 120)
          { 
              laser_angle_sum += pair.first;
              increment++;
          }
      }
      laser_angle_avg = laser_angle_sum/increment;
      laser_angle_avg = ((laser_angle_avg*M_PI)/180);
      bot->turnOdom(laser_angle_avg);
       bot->nextState = bot->States::DRIVE_FOWARD;
    }
    else {
      bot->nextState = bot->States::CONVEX_CORNER;
    }

    return;
}

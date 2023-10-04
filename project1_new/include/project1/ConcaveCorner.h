#ifndef CONCAVESTATE_H
#define CONCAVESTATE_H

#include "wallFollower.h"
#include <std_msgs/Int32.h>
#include <cmath>

class CConcaveCorner
{
    public:
        void handler(CWallFollower* bot);
    private:
        int MIN_BOUNDRY_ANGLE = 10;
        int MAX_BOUNDRY_ANGLE = 340;
        int MAX_BOUNDRY_DIA = 150;  //Cirucitlar Boundry of 150mm
        int TURTLEBOT_LIDAR_OFFSET_X = 0;
        int TURTLEBOT_LIDAR_OFFSET_Y = 0;
        double lidar_Call[2];

}

#endif //CONCAVESTATE_H
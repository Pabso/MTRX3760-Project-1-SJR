#ifndef CONCAVESTATE_H
#define CONCAVESTATE_H

#include "wallFollower.h"
#include <cmath>

#define DATA_READINGS 2

class CConcaveCorner
{
    public:
        void handler(CWallFollower* bot);
    private:
        void init(CWallFollower* bot);
        void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg);

        int MIN_BOUNDRY_ANGLE = 10;
        int MAX_BOUNDRY_ANGLE = 340;
        int MAX_BOUNDRY_DIA;  //Cirucitlar Boundry of 150mm
        int TURTLEBOT_LIDAR_OFFSET_X = 0;
        int TURTLEBOT_LIDAR_OFFSET_Y = 0.01;
        double lidar_Call[DATA_READINGS];

};

#endif //CONCAVESTATE_H
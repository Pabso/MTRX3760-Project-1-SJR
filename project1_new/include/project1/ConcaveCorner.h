#ifndef CONCAVESTATE_H
#define CONCAVESTATE_H

#include "wallFollower.h"
#include <cmath>

class CConcaveCorner
{
    public:
        void handler(CWallFollower* bot);
    private:
        void init(CWallFollower* bot);
        void laser_data_sraper(CWallFollower* bot);

        double laser_angle_sum;
        double laser_angle_avg;

};

#endif //CONCAVESTATE_H
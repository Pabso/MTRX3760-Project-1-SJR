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
        int delta_angle_comp(CWallFollower* bot);
        void laser_data_sraper(CWallFollower* bot);

        int laser_angle;

};

#endif //CONCAVESTATE_H
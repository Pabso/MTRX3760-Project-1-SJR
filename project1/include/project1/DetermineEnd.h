#ifndef DETERMINEEND_H
#define DETERMINEEND_H

#include "wallFollower.h"
#include <ros/ros.h>

class End
{
    public:
        void handler(CWallFollower *bot);
};

#endif // DETERMINEEND_H

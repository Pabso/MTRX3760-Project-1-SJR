#ifndef DETERMINEND_H
#define DETERMINEND_H

#include "wallFollower.h"
#include <ros/ros.h>

class End
{
    public:
    void handler(CWallFollower *bot);
};

#endif

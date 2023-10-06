#include "../include/project1/DetermineEnd.h"

void End::handler(CWallFollower *bot)
{   
    bot->nextState = bot->States::END;
}

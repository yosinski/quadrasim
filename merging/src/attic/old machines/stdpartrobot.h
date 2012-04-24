#ifndef _STDPARTROBOT_H
#define _STDPARTROBOT_H

//misc functions for creating robot from "standard" parts

#include "../physics.h"

void createRobot();
Part* createRobotPart( const NxMat34 &transform, float size);



#endif
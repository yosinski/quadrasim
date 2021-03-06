#ifndef _TESTOBJECTS_H
#define _TESTOBJECTS_H

#include "physx/MyCloth.h"

MyCloth* generateMuscle(NxVec3& pos, float height,float radius,int heightDivs,int aroundDivs);


void setupTestObjectScene();
void testObjectLoop();
void quadroLoop(const std::string &controlFileName, const std::string &logFileName,bool loop=true,bool useRealRobot=false);


#endif
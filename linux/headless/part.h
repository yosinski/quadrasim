#ifndef _PART_H
#define _PART_H

#include "NxPhysics.h"

class Part 
{
public:
	NxActor *act;

	Part(void);
	Part(NxActor *physicsEquivalent);

	void addTouchSensor(NxVec3 localPos,bool* statusMeter,float radius=0.1f);
};



#endif

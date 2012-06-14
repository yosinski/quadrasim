#include "part.h"
#include "NxPhysics.h"
#include "physics.h"
#include <assert.h>


Part::Part(void)
{
	act=NULL;
}

Part::Part(NxActor *physicsEquivalent)
{
	assert(physicsEquivalent && "part: physics equivalent invalid");
	this->act=physicsEquivalent;
}

void Part::addTouchSensor(NxVec3 localPos,bool* statusMeter,float radius)
{
	assert(act && "add touch sensor: physics equivalent invalid");
	NxSphereShapeDesc trig;
	trig.localPose.t=localPos;
	trig.radius=radius;
	trig.shapeFlags |= NX_TRIGGER_ENABLE;
	trig.userData=statusMeter;
	NxShape* trigShape=act->createShape(trig);
	assert(trigShape && "add touch sensor: could not create shape");
}	

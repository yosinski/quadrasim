#pragma once

#include "NxPhysics.h"

class DynamicActor;

class CylindricalJoint
{
public:
	CylindricalJoint(NxScene& scene, DynamicActor& actor1, DynamicActor& actor2, const NxVec3& globalAnchor, const NxVec3& globalAxis);

	void setTargetDistance(float d);

	void update(float simulationTime);

private:
	NxD6Joint* joint;
	DynamicActor& actor1;
	DynamicActor& actor2;
	NxVec3 actor1LocalAnchor;
	NxVec3 actor2LocalAnchor;
	float targetDistance;
};

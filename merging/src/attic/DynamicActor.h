#pragma once

#include "NxPhysics.h"

// to allow me to think in a more sensible orientation (x and y in the horizontal ground plane, z pointing to the sky)
NxVec3 layZAxisDown(const NxVec3& position);
NxMat33 layZAxisDown(const NxMat33& orientation);
NxVec3 liftZAxisUp(const NxVec3& position);
NxMat33 liftZAxisUp(const NxMat33& orientation);

NxMat33 makeOrientation(NxVec3 axis, NxVec3 normal);
NxMat33 makeOrientation(NxVec3 axis);


class DynamicActor {
public:
	NxActor& actor;

	DynamicActor(NxScene& scene, const NxVec3& position = NxVec3(0.0), const NxMat33& orientation = NxMat33(NX_IDENTITY_MATRIX));

	NxMat34 getZUpGlobalPose();
	NxVec3 transformToZUpGlobal(const NxVec3& x);

	NxVec3 transformFromGlobal(const NxVec3& x);

	NxVec3 getXAxis();
	NxVec3 getZAxis();

	void addBox(const NxVec3& position, const NxVec3& dimensions, const NxMat33& orientation = NxMat33(NX_IDENTITY_MATRIX));
	void addCapsule(const NxVec3& start, const NxVec3& end, float radius);
	void addCapsule(const NxVec3& start, float length, float radius, const NxMat33& orientation = NxMat33(NX_IDENTITY_MATRIX));

protected:
	void setGlobalPose(const NxVec3& position, const NxMat33& orientation);
};

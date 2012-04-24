#define _USE_MATH_DEFINES
#include <cmath>

#include "DynamicActor.h"

NxActor& createDynamicNxActor(NxScene& scene, const NxVec3& position, const NxMat33& orientation) {
	NxActorDesc actorDesc;
	actorDesc.globalPose = NxMat34(layZAxisDown(orientation), layZAxisDown(position));
	NxBodyDesc bodyDesc;
	//bodyDesc.angularDamping = .5;
	//bodyDesc.linearDamping = .1;
	bodyDesc.solverIterationCount = 60;
	actorDesc.body = &bodyDesc;
	// createActor() fails unless the actor has, e.g., a shape and a density (but it's no problem to remove it right away):
	NxBoxShapeDesc boxDesc;
	actorDesc.shapes.push_back(&boxDesc);
	actorDesc.density = 1;
	NxActor* actor = scene.createActor(actorDesc);
	actor->releaseShape(*actor->getShapes()[0]);
	return *actor;
}

DynamicActor::DynamicActor(NxScene& scene, const NxVec3& position, const NxMat33& orientation)
	: actor(createDynamicNxActor(scene, position, orientation)) {}

NxMat34 DynamicActor::getZUpGlobalPose() {return NxMat34(liftZAxisUp(actor.getGlobalOrientation()), liftZAxisUp(actor.getGlobalPosition())); }
NxVec3 DynamicActor::transformToZUpGlobal(const NxVec3& x) { return getZUpGlobalPose() * x; }

NxVec3 DynamicActor::transformFromGlobal(const NxVec3& x) { return actor.getGlobalPose() % x; }

NxVec3 DynamicActor::getXAxis() { return getZUpGlobalPose().M.getColumn(0); }
NxVec3 DynamicActor::getZAxis() { return getZUpGlobalPose().M.getColumn(2); }

void DynamicActor::addBox(const NxVec3& position, const NxVec3& dimensions, const NxMat33& orientation) {
	NxBoxShapeDesc desc;
	desc.localPose = NxMat34(orientation, position);
	desc.dimensions = dimensions;
	actor.createShape(desc);

	actor.updateMassFromShapes(1, 0);
}

void DynamicActor::addCapsule(const NxVec3& start, const NxVec3& end, float radius) {
	addCapsule(start, (end - start).magnitude(), radius, makeOrientation(end - start));
}

// its length extends (in one direction!) along the positive x-axis (of the given orientation) from the given starting point
// it is not centered about the start; it starts there.
void DynamicActor::addCapsule(const NxVec3& start, float length, float radius, const NxMat33& orientation) {
	NxVec3 midPoint = start + orientation.getColumn(0) * length / 2;
	NxMat33 rotation;  rotation.rotZ(-M_PI/2);  // to direct it along the x-axis

	NxCapsuleShapeDesc desc;
	desc.localPose = NxMat34(orientation * rotation, midPoint);
	desc.height = length;
	desc.radius = radius;
	actor.createShape(desc);

	if (!actor.updateMassFromShapes(1, 0))
		throw "updateMassFromShapes() failed";
}

void DynamicActor::setGlobalPose(const NxVec3& position, const NxMat33& orientation) {
	actor.setGlobalPose(NxMat34(layZAxisDown(orientation), layZAxisDown(position)));
}


NxVec3 layZAxisDown(const NxVec3& position) {
	NxMat33 rotation;  rotation.rotX(-M_PI/2);
	return rotation * position;
}
NxMat33 layZAxisDown(const NxMat33& orientation) {
	NxMat33 rotation;  rotation.rotX(-M_PI/2);
	return rotation * orientation;
}
NxVec3 liftZAxisUp(const NxVec3& position) {
	NxMat33 rotation;  rotation.rotX(M_PI/2);
	return rotation * position;
}
NxMat33 liftZAxisUp(const NxMat33& orientation) {
	NxMat33 rotation;  rotation.rotX(M_PI/2);
	return rotation * orientation;
}

NxMat33 makeOrientation(NxVec3 axis, NxVec3 normal) {
	axis.normalize();
	NxVec3 binormal = axis.cross(normal);  binormal.normalize();
	normal = binormal.cross(axis);
	NxMat33 orientation(axis, normal, binormal);  orientation.setTransposed();
	return orientation;
}
NxMat33 makeOrientation(NxVec3 axis) {
	axis.normalize();
	NxVec3 normal = axis.cross(NxVec3(0, 0, 1));
	if (normal.magnitude() < .1)
		normal = axis.cross(NxVec3(0, 1, 0));
	return makeOrientation(axis, normal);
}

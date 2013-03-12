#pragma once

#include "NxPhysics.h"

#include "../PhysX/DynamicActor.h"
#include "../PhysX/CylindricalJoint.h"

void addFixedJoint(NxScene& scene, DynamicActor& actor1, DynamicActor& actor2) {
	NxFixedJointDesc desc;
	desc.actor[0] = &actor1.actor;
	desc.actor[1] = &actor2.actor;
	scene.createJoint(desc);
}

void addRevoluteJoint(NxScene& scene, DynamicActor& actor1, DynamicActor& actor2, const NxVec3& globalAnchor, const NxVec3& globalAxis) {
	NxRevoluteJointDesc desc;
	desc.actor[0] = &actor1.actor;
	desc.actor[1] = &actor2.actor;
	desc.setGlobalAnchor(layZAxisDown(globalAnchor));
	desc.setGlobalAxis(layZAxisDown(globalAxis));

	desc.jointFlags |= NX_JF_COLLISION_ENABLED;
	//desc.solverExtrapolationFactor = 3;
	//desc.projectionMode = NX_JPM_POINT_MINDIST;
	//desc.projectionDistance = .05;
	scene.createJoint(desc);
}

void addIsoUniversalJoint(NxScene& scene, DynamicActor& actor1, DynamicActor& actor2, const NxVec3& globalAnchor, const NxVec3& globalAxis) {
	NxD6JointDesc desc;
	desc.actor[0] = &actor1.actor;
	desc.actor[1] = &actor2.actor;
	desc.setGlobalAnchor(layZAxisDown(globalAnchor));
	desc.setGlobalAxis(layZAxisDown(globalAxis));
	desc.xMotion = NX_D6JOINT_MOTION_LOCKED;
	desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	desc.zMotion = NX_D6JOINT_MOTION_LOCKED;
	desc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
	desc.swing1Motion = NX_D6JOINT_MOTION_FREE;
	desc.swing2Motion = NX_D6JOINT_MOTION_FREE;

	desc.jointFlags |= NX_JF_COLLISION_ENABLED;
	//desc.solverExtrapolationFactor = 3;
	//desc.projectionMode = NX_JPM_POINT_MINDIST;
	//desc.projectionDistance = .05;
	scene.createJoint(desc);
}

void addSphericalJoint(NxScene& scene, DynamicActor& actor1, DynamicActor& actor2, const NxVec3& globalAnchor) {
	NxSphericalJointDesc desc;
	desc.actor[0] = &actor1.actor;
	desc.actor[1] = &actor2.actor;
	desc.setGlobalAnchor(layZAxisDown(globalAnchor));

	desc.jointFlags |= NX_JF_COLLISION_ENABLED;
	//desc.solverExtrapolationFactor = 3;
	//desc.projectionMode = NX_JPM_POINT_MINDIST;
	//desc.projectionDistance = .05;
	scene.createJoint(desc);
}

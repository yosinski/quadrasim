#include "CylindricalJoint.h"
#include "DynamicActor.h"

CylindricalJoint::CylindricalJoint(NxScene& scene, DynamicActor& actor1_, DynamicActor& actor2_, const NxVec3& globalAnchor, const NxVec3& globalAxis)
	: targetDistance(0)
	, actor1(actor1_)
	, actor2(actor2_)
	, actor1LocalAnchor(actor1.transformFromGlobal(layZAxisDown(globalAnchor)))
	, actor2LocalAnchor(actor2.transformFromGlobal(layZAxisDown(globalAnchor))) {
	NxD6JointDesc desc;
	desc.actor[0] = &actor1.actor;
	desc.actor[1] = &actor2.actor;
	desc.setGlobalAnchor(layZAxisDown(globalAnchor));
	desc.setGlobalAxis(layZAxisDown(globalAxis));
	desc.xMotion = NX_D6JOINT_MOTION_FREE;
	desc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	desc.zMotion = NX_D6JOINT_MOTION_LOCKED;
	desc.twistMotion = NX_D6JOINT_MOTION_FREE;
	desc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	desc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

	desc.xDrive.driveType = NX_D6JOINT_DRIVE_VELOCITY;
	desc.xDrive.forceLimit = FLT_MAX;
	desc.xDrive.damping = 0;

	//desc.linearLimit.value=0;
	//desc.linearLimit.damping=FLT_MAX;
	//desc.linearLimit.restitution=0.0f;

	desc.jointFlags |= NX_JF_COLLISION_ENABLED;
	desc.solverExtrapolationFactor = 1.6;
	//desc.projectionMode = NX_JPM_POINT_MINDIST;
	//desc.projectionDistance = .05;
	joint = static_cast<NxD6Joint*>(scene.createJoint(desc));
}

void CylindricalJoint::setTargetDistance(float d) {
	targetDistance = d;
}

void CylindricalJoint::update(float simulationTime) {
	const float tolerance = 0.0;
	const float speed = 1;

	NxVec3 displacement = actor2.transformToZUpGlobal(actor2LocalAnchor) - actor1.transformToZUpGlobal(actor1LocalAnchor);
	NxVec3 normal = liftZAxisUp(joint->getGlobalAxis());  normal.normalize();
	float d = displacement.dot(normal);
	if(d < targetDistance - tolerance)
		joint->setDriveLinearVelocity(NxVec3(speed, 0, 0));
	else if(d > targetDistance + tolerance)
		joint->setDriveLinearVelocity(NxVec3(-speed, 0, 0));
	else
		joint->setDriveLinearVelocity(NxVec3(0, 0, 0));
}

#include "physics.h"
#include "physx/cooking.h"
#include <iostream>
#include <vector>
#include <boost/foreach.hpp>

NxPhysicsSDK* gPhysicsSDK = NULL;
NxScene* gScene = NULL;
ErrorStream gErrorStream;
bool gRenderUserData=false;
bool gCookingInitialized=false;
bool gHardwareSimulation=false;
bool gFreeze=false;

std::vector<Part *> parts;
std::vector<NxRevoluteJoint *> joints; 

const char* getNxSDKCreateError(const NxSDKCreateError& errorCode);


//trigger callback - used for contact sensors
class TriggerCallBack : public NxUserTriggerReport
{
	//quick solution: triggershape's userdata is a pointer to a boolean var
	void onTrigger(NxShape& triggerShape, NxShape& otherShape, NxTriggerFlag status) {
		if(status & NX_TRIGGER_ON_ENTER) {
			if(triggerShape.userData!=NULL)
				*((bool*)triggerShape.userData)=true;
		}
		if(status & NX_TRIGGER_ON_LEAVE) {
			if(triggerShape.userData!=NULL)
				*((bool*)triggerShape.userData)=false;
		}
	}
} myTriggerCallback;


void initPhysics()
{
	NxPhysicsSDKDesc desc;
	NxSDKCreateError errorCode = NXCE_NO_ERROR;
	gPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, NULL, NULL, desc, &errorCode);
	if(gPhysicsSDK == NULL) 
	{
		printf("\nSDK create error (%d - %s).\nUnable to initialize the PhysX SDK, exiting the sample.\n\n", errorCode, getNxSDKCreateError(errorCode));
		exit(1);
	}

	gPhysicsSDK->setParameter(NX_SKIN_WIDTH, 0.05f);

	//insert remote debug code here later
	// Set the debug visualization parameters
	gPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_CLOTH_MESH, 1);
	gPhysicsSDK->setParameter(NX_VISUALIZE_CLOTH_VALIDBOUNDS, 1);

	//check for HW
	NxHWVersion hwCheck = gPhysicsSDK->getHWVersion();
	if (hwCheck == NX_HW_VERSION_NONE) 
		gHardwareSimulation = false;
	//printf("HW simulation status: %d\n",gHardwareSimulation);

	//init cooking
	if (!gCookingInitialized) {
		gCookingInitialized = true;
		if (!InitCooking(NULL, &gErrorStream)) 
		  exit(1);
	}

	// Create a scene
	NxSceneDesc sceneDesc;
	
	//sceneDesc.simType=NX_SIMULATION_HW; //Only fluids/particles, cloth, and softbody are accelerated by GPU currently

	sceneDesc.gravity = NxVec3(0.0f, -9.81f, 0.0f);
	gScene = gPhysicsSDK->createScene(sceneDesc);
	if(gScene == NULL) 
	  exit(1);
	// Set default material
	NxMaterial* defaultMaterial = gScene->getMaterialFromIndex(0);

	defaultMaterial->setRestitution(0.0f);
	defaultMaterial->setStaticFriction(0.4f); //orig. quadro - but static is set up to dynamic
	defaultMaterial->setDynamicFriction(0.4f);

	//trigger callback
	gScene->setUserTriggerReport(&myTriggerCallback);

	// Create ground plane
	NxPlaneShapeDesc planeDesc;

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&planeDesc);
	gScene->createActor(actorDesc);
}


void terminatePhysics()
{
	BOOST_FOREACH(Part* p, parts)
		delete p;
	parts.clear();

	joints.clear();

	if(gCookingInitialized) 
		CloseCooking();
	gCookingInitialized = false;
	if(gPhysicsSDK != NULL)
	{
		if(gScene != NULL) gPhysicsSDK->releaseScene(*gScene);
		gScene = NULL;
		NxReleasePhysicsSDK(gPhysicsSDK);
		gPhysicsSDK = NULL;
	}


}


const char* getNxSDKCreateError(const NxSDKCreateError& errorCode) 
{
	switch(errorCode) 
	{
	case NXCE_NO_ERROR: return "NXCE_NO_ERROR";
	case NXCE_PHYSX_NOT_FOUND: return "NXCE_PHYSX_NOT_FOUND";
	case NXCE_WRONG_VERSION: return "NXCE_WRONG_VERSION";
	case NXCE_DESCRIPTOR_INVALID: return "NXCE_DESCRIPTOR_INVALID";
	case NXCE_CONNECTION_ERROR: return "NXCE_CONNECTION_ERROR";
	case NXCE_RESET_ERROR: return "NXCE_RESET_ERROR";
	case NXCE_IN_USE_ERROR: return "NXCE_IN_USE_ERROR";
	default: return "Unknown error";
	}
};





NxActor* createSphere(const NxVec3& pos, float size)
{
	if(gScene == NULL || !gScene->isWritable()) return NULL;	

	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc; //body desc required for dynamic actors
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius		= size;
	sphereDesc.localPose.t	= NxVec3(0, 0, 0);

	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 1.0f; //10??
	actorDesc.globalPose.t	= pos;	
	return gScene->createActor(actorDesc);
}

Part* createSpherePart(Point3D& pos, float size)
{
	Part* p=new Part(createSphere(NxVec3(pos.x,pos.y,pos.z),size));
	parts.push_back(p);
	return p;
}


NxActor* createBox(const NxVec3& pos, const NxVec3& dimensions,bool dynamic)
{
	if(gScene == NULL || !gScene->isWritable()) return NULL;	
	NxBodyDesc bodyDesc;
	bodyDesc.angularDamping	= 0.5f;
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions=dimensions;

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&boxDesc);
	if(dynamic)
		actorDesc.body=&bodyDesc;
	actorDesc.density=1.0f; //10 ?
	actorDesc.globalPose.t=pos;
	return gScene->createActor(actorDesc);
}

Part* createBoxPart(const Point3D& pos, Point3D& dimensions,bool dynamic)
{
	Part* p=new Part(createBox(NxVec3(pos.x,pos.y,pos.z),NxVec3(dimensions.x,dimensions.y,dimensions.z),dynamic));
	parts.push_back(p);
	return p;
}



NxActor* createCapsule(const NxVec3& pos, float length/*=1*/, bool centered/*=true*/, float radius )
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.radius		= radius;
	capsuleDesc.height		= length;
	if(centered)
		capsuleDesc.localPose.t = NxVec3(0, 0, 0);	//centered on actor 
	else
		capsuleDesc.localPose.t = NxVec3(0, length/2+capsuleDesc.radius, 0);	//capsule is on "floor"


	actorDesc.shapes.pushBack(&capsuleDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 1.0f; //10?
	actorDesc.globalPose.t	= pos;
	return gScene->createActor(actorDesc);
}

//remove point3d version soon
Part* createCapsulePart(const Point3D& pos, float length/*=1*/, bool centered/*=true*/, float radius)
{
	Part* p=new Part(createCapsule(NxVec3(pos.x,pos.y,pos.z),length,centered,radius));
	parts.push_back(p);
	return p;
}

Part* createCapsulePart1(NxVec3& pos, float length/*=1*/, bool centered/*=true*/, float radius)
{
	Part* p=new Part(createCapsule(pos,length,centered,radius));
	parts.push_back(p);
	return p;
}



NxRevoluteJoint* createRevoluteJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis,float limLowRad,float limHighRad,float spring,float damper)
{
	NxRevoluteJointDesc revDesc;

	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	revDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	revDesc.projectionMode = NX_JPM_POINT_MINDIST;
	revDesc.projectionDistance = 1.0f;
	revDesc.projectionAngle = 0.0872f;	//about 5 degrees in radians.
	

	//joint limit
	if(limLowRad!=0 || limHighRad!=0) {
		revDesc.flags |= NX_RJF_LIMIT_ENABLED;  //not jointFlags
		NxJointLimitPairDesc limDesc;
		limDesc.low.value=limLowRad;
		limDesc.high.value=limHighRad;
		revDesc.limit=limDesc;
	}

	//joint spring (motor)
	if(spring!=0 || damper!=0) {
		revDesc.flags |= NX_RJF_SPRING_ENABLED;
		NxSpringDesc springDesc;
		springDesc.targetValue=0;
		springDesc.spring=spring;
		springDesc.damper=damper;
		revDesc.spring=springDesc;
	}

	return (NxRevoluteJoint*)gScene->createJoint(revDesc);
}

NxRevoluteJoint* createRevoluteMotorJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis,float limLowRad,float limHighRad,float maxForce)
{
	//just add some spring to make it stiff before motor force is applied
	NxRevoluteJoint* j=createRevoluteJoint(a0,a1,globalAnchor,globalAxis,limLowRad,limHighRad,maxForce,maxForce);
	NxMotorDesc motor;
	motor.freeSpin=NX_FALSE;
	motor.maxForce=maxForce;
	motor.velTarget=0;
	j->setMotor(motor); //automatically enables motor
	//now we want to disable the motor until one starts controlling it, and use only a spring instead, for practical purposes
	NxU32 flags=j->getFlags();
	flags = flags ^ NX_RJF_MOTOR_ENABLED; //xor off the motor
	j->setFlags(flags);
	return j;
}

	
NxFixedJoint* createFixedJoint(NxActor* a0, NxActor* a1)
{
	NxRevoluteJointDesc revDesc;

	NxFixedJointDesc fixedDesc;
	fixedDesc.actor[0] = a0;
	fixedDesc.actor[1] = a1;

	return (NxFixedJoint*)gScene->createJoint(fixedDesc);
}


NxMat33 createRotationMatrix(float xAngle,float yAngle,float zAngle)
{
	NxMat33 xRot,yRot,zRot;
	xRot.rotX(xAngle);
	yRot.rotY(yAngle);
	zRot.rotZ(zAngle);
	NxMat33 res;
	res.multiply(xRot,yRot);
	res.multiply(res,zRot);
	return res;

}

NxMat34 createTfm(float x,float y,float z,float xAngle,float yAngle,float zAngle)
{
	NxMat34 tfm;
	tfm.M=createRotationMatrix(xAngle,yAngle,zAngle);
	tfm.t.set(x,y,z);
	return tfm;
}

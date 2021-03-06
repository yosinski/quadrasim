#ifndef _PHYSICS_H
#define _PHYSICS_H

#include "base/tools.h"
#include "base/kmath.h"
#include "NxPhysics.h" 
#include "NxMath.h" 
#include "physx/ErrorStream.h"
#include "part.h"
#include <vector>


extern NxPhysicsSDK*	gPhysicsSDK;
extern NxScene*			gScene;
extern bool gHardwareSimulation;
extern bool gRenderUserData;
extern bool gFreeze;

extern std::vector<Part *> parts;
extern std::vector<NxRevoluteJoint *> joints; //kan evt. bruke egen joint-klasse etterhvert


void initPhysics();
void terminatePhysics();

Part* createSpherePart(Point3D& pos, float size);
Part* createBoxPart(const Point3D& pos, const Point3D& dimensions,bool dynamic=true);
Part* createCapsulePart(const Point3D& pos, float length=1, bool centered=true, float radius=1);
Part* createCapsulePart1(NxVec3& pos, float length=1, bool centered=true, float radius=1);

NxRevoluteJoint* createRevoluteJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis,float limLowRad=0,float limHighRad=0,float spring=0,float damper=0);
NxRevoluteJoint* createRevoluteMotorJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis,float limLowRad=0,float limHighRad=0,float maxForce=1000);
NxFixedJoint* createFixedJoint(NxActor* a0, NxActor* a1); //could add breakable property here

NxMat33 createRotationMatrix(float xAngle,float yAngle,float zAngle); //rotation utility func
NxMat34 createTfm(float x=0,float y=0,float z=0,float xAngle=0,float yAngle=0,float zAngle=0); //transformation utility func



#endif

#include "stdpartrobot.h"
#include "../graphics/MeshGraphicsObject.h"
#include "../base/system.h"
#include "../graphics/glstuff.h"
#include "../physics.h"


MeshGraphicsObject *robMesh;
Part* createRobotPart( const NxMat34 &transform, float size)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	float rad=1.0f;

	NxCapsuleShapeDesc cd;
	cd.radius		= rad;
	cd.height		= size;
	CapsuleGraphicsObject *cg=new CapsuleGraphicsObject(rad,size);  //this creates a lot of duplicates, doesn't matter at the moment
	cd.userData=cg;
	cd.localPose.t = NxVec3(0, 0, 0);	//capsule is on "floor"
	actorDesc.shapes.pushBack(&cd);

	NxCapsuleShapeDesc cd2;
	cd2.radius		= rad;
	cd2.height		= size;
	CapsuleGraphicsObject *cg2=new CapsuleGraphicsObject(rad,size);  //this creates a lot of duplicates, doesn't matter at the moment
	cd2.userData=cg2;
	cd2.localPose.t = cd.localPose.t+NxVec3(2*rad, 0, 0);	
	actorDesc.shapes.pushBack(&cd2);

	//perpendicular
	NxCapsuleShapeDesc cd3;
	cd3.radius		= rad;
	cd3.height		= size;
	CapsuleGraphicsObject *cg3=new CapsuleGraphicsObject(cd3.radius,cd3.height);  //this creates a lot of duplicates, doesn't matter at the moment
	cd3.userData=cg3;
	cd3.localPose.t = cd2.localPose.t+NxVec3(size/2, -size/2+0.3f*size, 0);	//capsule is on "floor"
	cd3.localPose.M.id();
	cd3.localPose.M.rotZ(-PI/2.0); //in radians, not angles like doc says
	actorDesc.shapes.pushBack(&cd3);


	actorDesc.body=&bodyDesc;
	actorDesc.density=10.0f;
	actorDesc.globalPose=transform;

	if(NULL==robMesh) {
		robMesh=new MeshGraphicsObject("data/untitled2.obj");
	}

	//scale down object and orient it according to the shapes.. could have done it the other way round also
	glPushMatrix();
	glLoadIdentity();
	float scale=0.026f;
	glTranslatef(5.2f,-2.4f,0); 
	glRotatef(90,1,0,0);
	glRotatef(90,0,1,0);
	glScalef(scale,scale,scale);
	glGetFloatv(GL_MODELVIEW_MATRIX,robMesh->transform);
	glPopMatrix();


	actorDesc.userData=robMesh;
	//return gScene->createActor(actorDesc);
	NxActor* actor=gScene->createActor(actorDesc);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	return p;
}



void createRobot()
{
	float length=4.0;
	float rad=1.0f;
	Part *lastPart;

	//float x,y,z;
	//x=y=z=0;
	NxMat34 tfm0;
	tfm0.id();
	tfm0.M.rotX(-PI/2);
	tfm0.t.set(0,20,0);
	Part *p0=createRobotPart(tfm0,length); 
	lastPart=p0;

	for(int seg=1;seg<5;seg++) {

		//somewhat hairy local transformation (relative to previous part)
		NxMat34 localTransform;
		localTransform.id();
		localTransform.t.set(2*rad+length+length*0.05f,-length/2+0.3f*length-2*rad,0); //disse verdiene må bare justeres senere
		NxMat33 xrot,zrot;
		xrot.rotX(PI);
		zrot.rotZ(-PI/2);
		localTransform.M.multiply(zrot,xrot);

		//get world transformation by multiplying with global pose from previous part
		NxMat34 globalTransform=lastPart->act->getGlobalPose();
		globalTransform.multiply(globalTransform,localTransform);

		//link anchor point
		NxVec3 localLinkPos;
		localLinkPos.set(localTransform.t);
		NxVec3 globalLinkPos=lastPart->act->getGlobalPose()*localLinkPos;

		//link rotation axis
		NxVec3 localLinkAxis(1,0,0);
		NxVec3 globalLinkAxis;
		lastPart->act->getGlobalPose().M.multiply(localLinkAxis,globalLinkAxis);

		Part* p=createRobotPart(globalTransform,length);

		NxRevoluteJoint *j=createRevoluteJoint(lastPart->act,p->act,globalLinkPos,globalLinkAxis);
		if(j==NULL) systemError("funka ikke");
		joints.push_back(j);
		lastPart=p;

	}
}


#include "x2machine.h"

#include <assert.h>
#include <stdlib.h>
#include "../physics.h"
#include "../graphics/MeshGraphicsObject.h"
#include "../base/system.h"

Part* X2Machine::createCylinderCore(NxActor* attachTo, NxMat34& transform)
{
	NxMat34 parentTransform;
	if(attachTo)
		parentTransform=attachTo->getGlobalPose();
	else
		parentTransform.id();

	NxMat34 resTfm;
	resTfm.multiply(parentTransform,transform);


	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	actorDesc.body=&bodyDesc;
	actorDesc.globalPose=resTfm;
	//actorDesc.density=1.0f; 
	actorDesc.density=0.3f; //hollow
	actorDesc.userData=m_outerRing;

	NxBoxShapeDesc vertBox;
	vertBox.dimensions.set(m_size*0.6f,m_size*1.15f,m_size*0.6f);
	vertBox.localPose.t.set(0,0,0);
	actorDesc.shapes.push_back(&vertBox);

	NxMat33 rot;
	rot.rotX(PI/2);
	NxBoxShapeDesc horBox;
	horBox.dimensions.set(m_size*0.6f,m_size*1.15f,m_size*0.6f);
	horBox.localPose.t.set(0,0,0);
	horBox.localPose.M=rot;
	actorDesc.shapes.push_back(&horBox);

	NxMat33 rot2;
	rot.rotZ(PI/2);
	NxBoxShapeDesc electroBox;
	electroBox.dimensions.set(m_size*0.75f,m_size*1.1f,m_size*0.75f);
	electroBox.localPose.t.set(0,0,0);
	electroBox.localPose.M=rot;
	actorDesc.shapes.push_back(&electroBox);

	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	return p;	
}

Part* X2Machine::createArm(NxActor* attachTo, NxMat34& transform)
{
	NxMat34 parentTransform;
	if(attachTo)
		parentTransform=attachTo->getGlobalPose();
	else
		parentTransform.id();

	NxMat34 resTfm;
	resTfm.multiply(parentTransform,transform);


	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	actorDesc.body=&bodyDesc;
	actorDesc.globalPose=resTfm;
	actorDesc.density=1.0f;
	actorDesc.userData=m_arm;

	float width=m_size*0.8f;
	float height=m_size*1.75f;
	float depth=m_size*0.15f;
	float zOffs=m_size*1.25f+depth;

	NxBoxShapeDesc box1;
	box1.dimensions.set(width,height,depth);
	box1.localPose.t.set(0,height,zOffs);
	actorDesc.shapes.push_back(&box1);

	NxMat33 rot;
	NxBoxShapeDesc box2;
	box2.dimensions.set(width,height,depth);
	box2.localPose.t.set(0,height,-zOffs);
	actorDesc.shapes.push_back(&box2);


	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	return p;	

}


Part* X2Machine::createBase(NxActor* attachTo, NxMat34& transform)
{
	NxMat34 parentTransform;
	if(attachTo)
		parentTransform=attachTo->getGlobalPose();
	else
		parentTransform.id();

	NxMat34 resTfm;
	resTfm.multiply(parentTransform,transform);

	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	actorDesc.body=&bodyDesc;
	actorDesc.globalPose=resTfm;
	actorDesc.density=1.0f;
	actorDesc.userData=m_base;

	float width=m_size*1.1f;
	float height=m_size*0.1f;
	float depth=m_size*1.1f;

	NxBoxShapeDesc box1;
	box1.dimensions.set(width,height,depth);
	box1.localPose.t.set(0,height,0);
	actorDesc.shapes.push_back(&box1);

	NxBoxShapeDesc box2;
	box2.dimensions.set(width,height,depth);
	box2.localPose.t.set(0,height,0);
	box2.localPose.M.rotY(PI/4);
	actorDesc.shapes.push_back(&box2);

	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	return p;	
}

Part* X2Machine::createTip(NxActor* attachTo, NxMat34& transform)
{
	NxMat34 parentTransform;
	if(attachTo)
		parentTransform=attachTo->getGlobalPose();
	else
		parentTransform.id();

	NxMat34 resTfm;
	resTfm.multiply(parentTransform,transform);

	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	actorDesc.body=&bodyDesc;
	actorDesc.globalPose=resTfm;
	actorDesc.density=1.0f;
	actorDesc.userData=m_tip;

	float height=m_size*1.0f;
	float rad=m_size*0.3f;

	NxCapsuleShapeDesc cap1;
	cap1.height=height;
	cap1.radius=rad;
	cap1.localPose.t.set(0,height/2+rad,0);
	actorDesc.shapes.push_back(&cap1);

	NxCapsuleShapeDesc cap2;
	cap2.height=height*6;
	cap2.radius=rad;
	cap2.localPose.t.set(0,height+rad,0);
	cap2.localPose.M.rotX(PI/2);
	actorDesc.shapes.push_back(&cap2);
	


	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	return p;	

}



void X2Machine::initMeshes()
{
	//mesh loading and transformation
	m_size=5.0f;
	m_outerRing=new MeshGraphicsObject("data/x2/ytrering");
	Vec3 minPos,maxPos;
	m_outerRing->getExtents(minPos,maxPos);
	glPushMatrix();
	glLoadIdentity();
	float scale=m_size*2.0f/(maxPos.x-minPos.x);
	//glTranslatef(5.2f,-2.4f,0); 
	//glRotatef(90,1,0,0);
	glScalef(scale,scale,scale);
	glGetFloatv(GL_MODELVIEW_MATRIX,m_outerRing->transform);
	glPopMatrix();

	glPushMatrix();
	glLoadIdentity();
	glScalef(scale,scale,scale);
	m_arm=new MeshGraphicsObject("data/x2/arm");
	glGetFloatv(GL_MODELVIEW_MATRIX,m_arm->transform);
	m_base=new MeshGraphicsObject("data/x2/sokkel");
	glGetFloatv(GL_MODELVIEW_MATRIX,m_base->transform);
	m_tip=new MeshGraphicsObject("data/x2/tut");
	glGetFloatv(GL_MODELVIEW_MATRIX,m_tip->transform);


	glPopMatrix();

}

X2Machine::X2Machine(X2Params* params/* =NULL */)
{
	if(NULL==params)
		params=new X2Params();
	assert(params);
	params->print();
	m_params=params;

	//FIX: need to resolve if params should be pointer or someth else.
	//does it copy with maps and vectors automatically?

	initMeshes();

	Part* lastPart=NULL;
	Part* curPart=NULL;
	NxRevoluteJoint* joint=NULL;
	float limLow=0;
	float limHigh=0;
	float limAmp=0;
	float initAngle=0;
	float paramLow=0,paramHigh=0;
	//float spring=500000;
	float spring=5000000;
	float damper= 500000.0f;
	NxMat34 tfm;
	NxVec3 globalAxis;

/*
	//temp test:
	//arm
	tfm.t.set(0,5,0);
	curPart=createArm(NULL,tfm);
	m_robParts.push_back(curPart);
*/




	//base
	tfm.t.set(0,0,0);
	tfm.M.id();
	curPart=createBase(NULL,tfm);
	m_robParts.push_back(curPart);
	//createFixedJoint(NULL,curPart->act);  //fixes it to the ground
	lastPart=curPart;

	//core 1
	tfm.t.set(0,6,0);
	tfm.M.id();
	curPart=createCylinderCore(lastPart->act,tfm);
	m_robParts.push_back(curPart);
	createFixedJoint(lastPart->act,curPart->act);
	lastPart=curPart;

	//core 2
	paramLow=params->getValue("limLow",0);
	paramHigh=params->getValue("limHigh",0);
	limAmp=(paramHigh+paramLow)/2*PI;
	initAngle=(-paramLow+paramHigh)/2*PI;
	tfm.M=createRotationMatrix(PI/2,0,initAngle);
	tfm.t.set(0,12,0);
	curPart=createCylinderCore(lastPart->act,tfm);
	m_robParts.push_back(curPart);
	lastPart->act->getGlobalOrientation().multiply(NxVec3(0,1,0),globalAxis);
	joint=createRevoluteJoint(lastPart->act,curPart->act,lastPart->act->getGlobalPose().t,globalAxis,-limAmp,limAmp,spring,damper);
	m_robJoints.push_back(joint);
	lastPart=curPart;

	//arm
	paramLow=params->getValue("limLow",1);
	paramHigh=params->getValue("limHigh",1);
	limAmp=(paramHigh+paramLow)/2*PI;
	initAngle=(-paramLow+paramHigh)/2*PI;
	tfm.M=createRotationMatrix(-PI/2,0,initAngle);
	tfm.t.set(0,0,0);
	curPart=createArm(lastPart->act,tfm);
	m_robParts.push_back(curPart);
	lastPart->act->getGlobalOrientation().multiply(NxVec3(0,1,0),globalAxis);
	joint=createRevoluteJoint(lastPart->act,curPart->act,lastPart->act->getGlobalPose().t,globalAxis,-limAmp,limAmp,spring,damper);
	m_robJoints.push_back(joint);
	lastPart=curPart;

	//core 3
	paramLow=params->getValue("limLow",2);
	paramHigh=params->getValue("limHigh",2);
	limAmp=(paramHigh+paramLow)/2*PI;
	initAngle=(-paramLow+paramHigh)/2*PI;
	tfm.M=createRotationMatrix(PI/2,initAngle,0);
	tfm.t.set(0,17,0);
	curPart=createCylinderCore(lastPart->act,tfm);
	m_robParts.push_back(curPart);
	curPart->act->getGlobalOrientation().multiply(NxVec3(0,1,0),globalAxis);
	joint=createRevoluteJoint(lastPart->act,curPart->act,curPart->act->getGlobalPose().t,globalAxis,-limAmp,limAmp,spring,damper);
	m_robJoints.push_back(joint);
	lastPart=curPart;

	//tip
	tfm.t.set(0,0,-5);
	tfm.M.rotX(-PI/2);
	curPart=createTip(lastPart->act,tfm);
	m_robParts.push_back(curPart);
	createFixedJoint(lastPart->act,curPart->act);
	lastPart=curPart;


	//quick hack to increase stability
	int solverIterations=10; //4 is default
	for each(Part* part in m_robParts)
		part->act->setSolverIterationCount(solverIterations);


}


void X2Machine::update(float simulationTime)
{
	//temp hacking now
	//float f=sin(simulationTime/10.0f)*0.8f*PI;

	//need to correlate this more closely to motors, should not have speeds exceeding motor max speed.
	//is it ok to set position directly?
	
	float phase=0;
	int i=0;
	for each(NxRevoluteJoint* j in m_robJoints) {
		NxSpringDesc sd;
		j->getSpring(sd);
		NxJointLimitPairDesc lim;
		j->getLimits(lim);
		float freq=m_params->getValue("freq",i);
		float f=sin(freq*simulationTime+phase*PI)*lim.high.value*0.9f; //should not go all the way out(?)
		sd.targetValue=f;
		j->setSpring(sd);
		phase+=0.3f; //FIX: get from params
		i++;
	}


	

}
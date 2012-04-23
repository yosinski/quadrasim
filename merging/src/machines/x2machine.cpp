#include "x2machine.h"

#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#define cout STD_COUT

#include <assert.h>
#include <stdlib.h>
#include "../physics.h"
#include "../graphics/MeshGraphicsObject.h"
#include "../base/system.h"
#include "../graphics/graphics.h"

//these are class static to speed up loading time
MeshGraphicsObject* X2Machine::m_outerRing=NULL;
MeshGraphicsObject* X2Machine::m_arm=NULL;
MeshGraphicsObject* X2Machine::m_base=NULL;
MeshGraphicsObject* X2Machine::m_tip=NULL;
MeshGraphicsObject* X2Machine::m_lid=NULL;
MeshGraphicsObject* X2Machine::m_encBox=NULL;
MeshGraphicsObject* X2Machine::m_platform=NULL;
MeshGraphicsObject* X2Machine::m_gripTip=NULL;

#define PRINT_STATS 1 //for printing controller and position stats

Part* X2Machine::createCylinderCore(NxActor* attachTo, NxMat34& transform, bool glue)
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
	//vertBox.dimensions.set(m_size*0.6f,m_size*1.15f,m_size*0.6f);
	vertBox.dimensions.set(m_size*0.6f,m_size*1.12f,m_size*0.6f);
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
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;
}

Part* X2Machine::createArm(NxActor* attachTo, NxMat34& transform, bool glue, float armLength)
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

	//m_sz=7.14f
	float width=m_size*0.8f;
	//float height=m_size*1.75f;
	//float height=X2_ARMLEN;
	float height=armLength;
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
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;	
}


Part* X2Machine::createBase(NxActor* attachTo, NxMat34& transform, bool glue)
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
	float height=m_size*0.18f;
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
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;	
}

Part* X2Machine::createTip(NxActor* attachTo, NxMat34& transform, bool glue,float tipLength)
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

	//float height=8.5f;
	float height=tipLength;
	float rad=2.8f;

	//exp: longer height + friction
	NxCapsuleShapeDesc cap1;
	cap1.height=height;
	cap1.radius=rad;
	cap1.localPose.t.set(0,height/2+rad,0);
	actorDesc.shapes.push_back(&cap1);

	//high friction on the tip
	NxBoxShapeDesc box1;
	box1.dimensions.set(rad/2,rad/2,rad/2); //88 10 88
	box1.localPose.t.set(0,height+2*rad-rad/2,0);
	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0.0f;
	materialDesc.staticFriction = 2.0f;
	materialDesc.dynamicFriction = 2.0f;
	NxMaterial *mat=gScene->createMaterial(materialDesc);
	box1.materialIndex= mat->getMaterialIndex();
	actorDesc.shapes.push_back(&box1);


	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;	
}

Part* X2Machine::createGripTip(NxActor* attachTo, NxMat34& transform, bool glue)
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
	actorDesc.userData=m_gripTip;


	float rad=3.0f;
	//float height=23.0f; //?
	float height=21.0f; //?
	NxCapsuleShapeDesc cap1;
	cap1.height=height;
	cap1.radius=rad;
	cap1.localPose.t.set(0,height/2+rad,0);
	actorDesc.shapes.push_back(&cap1);

	NxBoxShapeDesc box1;
	box1.dimensions.set(4.4f,0.5f,4.4f); //88 10 88
	//box1.localPose.t.set(0,29.0f,0);
	box1.localPose.t.set(0,27.0f,0);
	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0.0f;
	//materialDesc.staticFriction = 1.0f;
	//materialDesc.dynamicFriction = 1.0f;
	materialDesc.staticFriction = 5.0f;
	materialDesc.dynamicFriction = 5.0f;
	NxMaterial *mat=gScene->createMaterial(materialDesc);
	box1.materialIndex= mat->getMaterialIndex();
	actorDesc.shapes.push_back(&box1);

	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;	
}


Part* X2Machine::createLid(NxActor* attachTo, NxMat34& transform, bool glue)
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
	actorDesc.userData=m_lid;

	float height=m_size*1.0f;

	NxBoxShapeDesc box1;
	box1.dimensions.set(height*0.1f,height,height);
	//box1.localPose.t.set(0,height,0);
	box1.localPose.t.set(0,0,0);
	actorDesc.shapes.push_back(&box1);

	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;	
}

Part* X2Machine::createEncoderBox(NxActor* attachTo, NxMat34& transform, bool glue)
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
	actorDesc.userData=m_encBox;

	float height=m_size*0.8f;
	float width=m_size*0.2f;

	NxBoxShapeDesc box1;
	box1.dimensions.set(width,height,height);
	//box1.localPose.t.set(0,height,0);
	box1.localPose.t.set(width,0,0);
	actorDesc.shapes.push_back(&box1);

	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;	
}


Part* X2Machine::createPlatform(NxActor* attachTo, NxMat34& transform, bool glue)
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
	actorDesc.userData=m_platform;

	NxBoxShapeDesc box1;
	//box1.dimensions.set(width*4,height*1,depth*2.5f);
	box1.dimensions.set(30,2.5f/2,20); //600, 25, 400
	box1.localPose.t.set(-18,2.5f/2,0);
	box1.density=2.5f;
	NxMaterialDesc materialDesc;
	materialDesc.restitution = 0.0f;
	materialDesc.staticFriction = 0.1f;
	materialDesc.dynamicFriction = 0.1f;
	materialDesc.staticFriction = 0.3f;
	materialDesc.dynamicFriction = 0.3f;
	NxMaterial *mat=gScene->createMaterial(materialDesc);
	box1.materialIndex= mat->getMaterialIndex();
	actorDesc.shapes.push_back(&box1);

	NxActor* actor=gScene->createActor(actorDesc);
	assert(actor);
	Part *p=new Part();
	p->act=actor;
	parts.push_back(p);
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;
}


//mesh loading and transformation
void X2Machine::initMeshes()
{
	m_size=7.14f; //this is a bit arbitrary now, try to avoid m_size from now on
	if(m_outerRing==NULL) { //(static) meshes not loaded yet
		//m_outerRing=new MeshGraphicsObject("data/x2/ytrering");
		m_outerRing=new MeshGraphicsObject("data/x2/ytrering_lo");
		m_outerRing->m_mat->setColor(0.9f,0.9f,0.9f);
		float scale=0.1f; //units in mm from solidworks, lets us work in cm
		glPushMatrix();
		glLoadIdentity();
		glScalef(scale,scale,scale);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_outerRing->transform);
		//m_arm=new MeshGraphicsObject("data/x2/arm");
		m_arm=new MeshGraphicsObject("data/x2/arm_lo");
		m_arm->m_mat->setColor(1,1,1);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_arm->transform);
		//m_base=new MeshGraphicsObject("data/x2/sokkel");
		m_base=new MeshGraphicsObject("data/x2/sokkel_lo");
		m_base->m_mat->setColor(1,1,1);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_base->transform);
		m_tip=new MeshGraphicsObject("data/x2/tut");
		m_tip->m_mat->setColor(1,0.3f,0.3f);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_tip->transform);

		//m_lid=new MeshGraphicsObject("data/x2/lokk");
		m_lid=new MeshGraphicsObject("data/x2/lokk_lo");
		//m_lid->m_mat->setColor(0,0,0.8f);
		m_lid->m_mat->setColor(0.2f,0.2f,0.4f);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_lid->transform);

		//m_encBox=new MeshGraphicsObject("data/x2/encboks");
		m_encBox=new MeshGraphicsObject("data/x2/encboks_lo");
		m_encBox->m_mat->setColor(0.2f,0.2f,0.4f);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_encBox->transform);

		m_platform=new MeshGraphicsObject("data/x2/platform");
		m_platform->m_mat->setColor(0.2f,0.2f,0.4f);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_platform->transform);

		m_gripTip=new MeshGraphicsObject("data/x2/gripeboks");
		m_gripTip->m_mat->setColor(0.2f,0.2f,0.4f);
		glGetFloatv(GL_MODELVIEW_MATRIX,m_gripTip->transform);

		glPopMatrix();
	}
}

void controlMotor(NxRevoluteJoint* joint, float targetAngle)
{
	//float p=0.1f;
	float p=2.5f;
	float maxSpeed=0.5f;
	//float maxSpeed=1.5f;
	float curAngle=joint->getAngle();
	float deltaAngle=targetAngle-curAngle;
	float desiredSpeed=deltaAngle*p;
	desiredSpeed=__min(__max(-maxSpeed,desiredSpeed),maxSpeed); //clamping

	NxMotorDesc motor;
	joint->getMotor(motor);
	motor.velTarget=desiredSpeed;
	joint->setMotor(motor); //automatically enables motor
}


X2Machine::X2Machine()
{
	m_params=NULL;
	initMeshes();
	m_lidOffset=5.7f*1.41f;
	m_encBoxOffset=5.0f*1.41f;
	m_coreOffset=16.8f;


}



NxRevoluteJoint* X2Machine::createParamMotorJoint(Part* p1,NxVec3 p1LocalAxis,Part* p2,int jointParamNumber)
{
	float maxForce=1000000;
	NxRevoluteJoint* joint;
	NxVec3 globalAxis;
	float paramLow=m_params->getValue("limLow",jointParamNumber);
	float paramHigh=m_params->getValue("limHigh",jointParamNumber);
	float limAmp=(paramHigh+paramLow)/2*PI;

	p1->act->getGlobalOrientation().multiply(p1LocalAxis,globalAxis);
	joint=createRevoluteMotorJoint(p1->act,p2->act,p1->act->getGlobalPose().t,globalAxis,-limAmp,limAmp,maxForce);
	m_robJoints.push_back(joint);
	return joint;
}


float X2Machine::getInitAngle(int jointNumber)
{
	float paramLow=m_params->getValue("limLow",jointNumber);
	float paramHigh=m_params->getValue("limHigh",jointNumber);
	float initAngle=(-paramLow+paramHigh)/2*PI;
	return initAngle;
}




NxVec3 X2Machine::getPosition()
{
	NxVec3 pos(0,0,0);
	if(m_robParts[0]) 
		pos=m_robParts[0]->act->getGlobalPosition();
	return pos;
}

void X2Machine::update(float simulationTime)
{
	//also: check periods and curves of control function.
	//will it saturate, resulting in too quick movements?
	//plot or print curve (or at least t0, t1, ...)
	static FILE* controllerOut=NULL;
	if(PRINT_STATS) {
		if(controllerOut==NULL) {
			controllerOut=fopen("sin_controller_out.txt","w");
			fprintf(controllerOut,"#simtime (jointctrlval(normalized) jointctrlval actualjointangle)x%d xpos ypos absdeltapos\n",m_robJoints.size());

		}
		fprintf(controllerOut,"%.3f ",simulationTime);
	}

	int i=0;
	for each(NxRevoluteJoint* j in m_robJoints) {
		//NxSpringDesc sd;
		//j->getSpring(sd);
		NxJointLimitPairDesc lim;
		j->getLimits(lim); //lim.high and low are equal, initial rotation decided from parameters limLow and limHigh
		//float freq=m_params->getValue("freq",i);
		float freq=m_params->getValue("freq");
		float phase=m_params->getValue("phase",i); //could also have accumulated phase.. more interesting for other machines?

		//float f=sin(freq*simulationTime+phase*PI)*lim.high.value*0.9f; //should not go all the way out(?)
		//sd.targetValue=f;

		float attackTime=m_params->getValue("attack",i);
		float p0Time=m_params->getValue("p0",i);
		float decayTime=m_params->getValue("decay",i);
		float p1Time=m_params->getValue("p1",i);
		float period=attackTime+p0Time+decayTime+p1Time;
		attackTime/=period;
		p0Time/=period;
		decayTime/=period;
		p1Time/=period;
		float t0=0;
		float t1=attackTime;
		float t2=t1+p0Time;
		float t3=t2+decayTime;
		float t=freq*simulationTime;
		t=__max(t-phase,0); //subtract phase but wait in the beginning so no abrupt movements
		t+=t1/2.0f; //want to start with muscles in neutral position

		float remainder=fmod(t,1.0f);
		float angularPos=sineEnvelope(remainder,t0,t1,t2,t3)*2-1;
		if(PRINT_STATS) fprintf(controllerOut,"%.3f ",angularPos); //normalized controller out

		angularPos*=lim.high.value*0.9f; //should not go all the way out(?)
		if(PRINT_STATS) { 
			fprintf(controllerOut,"%.3f ",angularPos); //adjusted for angle limits
			float actualAngle=j->getAngle();
			fprintf(controllerOut,"%.3f ",actualAngle); 
		}

		//sd.targetValue=angularPos;
		//j->setSpring(sd);
		controlMotor(j,angularPos);
		
		i++;
	}



	if(PRINT_STATS) {
		static NxVec3 lastPos=getPosition();
		NxVec3 curPos=getPosition();
		float xPos=curPos.x;
		float yPos=curPos.y;
		float absSpeed=(curPos-lastPos).magnitude();
		lastPos=curPos;
		fprintf(controllerOut,"%.3f %.3f %.3f ",xPos,yPos,absSpeed); //xpos,ypos, abs speed
	}

	//hack: all y positions
	//int numParts=m_robParts.size();
	/*if(PRINT_STATS) { 
		for each(Part* p in m_robParts) {
			float y=p->act->getGlobalPosition().y;
			fprintf(controllerOut,"%.2f ",y); 
		}
	}*/

	if(PRINT_STATS) {
		fprintf(controllerOut,"\n");
	}

}


Part* X2Machine::getCentralPart()
{
	if(m_robParts[0])
		return m_robParts[0];
	else
		return NULL;
}



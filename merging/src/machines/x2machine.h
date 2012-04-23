#ifndef X2MACHINE_H
#define X2MACHINE_H

//base class for x2 configurations

#include "machine.h"
#include "../base/floatparammultivallist.h"
#include "../part.h"
#include "../graphics/MeshGraphicsObject.h"
#include <vector>
#include "../evolutionharness.h"

#define X2_TIPLEN 8.5f
//#define X2_ARMLEN 12.495f
#define X2_ARMLEN 12.0f

//////////////////////////////////////////////////////////////////////////

class X2Machine : public Machine
{
public:
	//X2Machine(X2Params* params=NULL);
	X2Machine();
	void update(float simulationTime);
	NxVec3 getPosition(void);
	Part* getCentralPart(void);

protected:
	Part* createPlatform(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createBase(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createCylinderCore(NxActor* attachTo, NxMat34& transform, bool glue);
	//Part* createArm(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createArm(NxActor* attachTo, NxMat34& transform, bool glue,float armLength=X2_ARMLEN);
	Part* createTip(NxActor* attachTo, NxMat34& transform, bool glue,float tipLength=X2_TIPLEN);
	Part* createGripTip(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createLid(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createEncoderBox(NxActor* attachTo, NxMat34& transform, bool glue);
	void initMeshes();
	float getInitAngle(int jointNumber);
	//NxRevoluteJoint* createParamMotorJoint(Part* part1, Part* part2,int jointNumber);
	NxRevoluteJoint* createParamMotorJoint(Part* p1,NxVec3 p1LocalAxis,Part* p2,int jointParamNumber);

	float m_size;
	std::vector<Part*> m_robParts;
	std::vector<NxRevoluteJoint*> m_robJoints; //only the revolute motors
	static MeshGraphicsObject* m_outerRing;
	static MeshGraphicsObject* m_arm;
	static MeshGraphicsObject* m_base;
	static MeshGraphicsObject* m_tip;
	static MeshGraphicsObject* m_gripTip;
	static MeshGraphicsObject* m_lid;
	static MeshGraphicsObject* m_encBox;
	static MeshGraphicsObject* m_platform;

	float m_lidOffset;
	float m_encBoxOffset;
	float m_coreOffset;

	FloatParamMultiValList *m_params; //used by all subclasses
};



#endif
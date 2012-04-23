#ifndef X2MACHINE_H
#define X2MACHINE_H

//#include "../base/floatparamlist.h"
#include "../base/floatparammultivallist.h"
#include "../part.h"
#include "../graphics/MeshGraphicsObject.h"
#include <vector>

//struct X2Params : public FloatParamList
struct X2Params : public FloatParamMultiValList
{
	X2Params::X2Params() {
		
		//deduce initAngle and limitAmp from limLow and limHigh, easier for control system
		//but keep limlow/high as parameters because easiest to specify joint min/max

		int numJoints=3;
		//setParams("limLow",FloatParam(0.5f,0,1,6),numJoints);
		//paramList["limLow"][1]=FloatParam(0.5f,0,0.8f,6); //hardcode way of changing a param in the list
		//alternatively: use addparam repetitively
		addParam("limLow",FloatParam(0.5f,0,1,6));
		addParam("limLow",FloatParam(0.5f,0,0.6f,6));
		addParam("limLow",FloatParam(0.5f,0,0.6f,6));

		//setParams("limHigh",FloatParam(0.5f,0,1,6),numJoints);
		addParam("limHigh",FloatParam(0.3f,0,1,6));
		addParam("limHigh",FloatParam(0.5f,0,0.6f,6));
		addParam("limHigh",FloatParam(0.3f,0,0.6f,6));


		addParam("freq",FloatParam(0.3f,0,1));
		addParam("freq",FloatParam(0.4f,0,1));
		addParam("freq",FloatParam(0.5f,0,1));


		setParams("phase",FloatParam(0.0f,0,1),numJoints);
		setParams("attack",FloatParam(0.25f,0,0.8f),numJoints); //not sure how to fix this so that all combinations will be smooth
		setParams("p0",FloatParam(0.25f,0,0.8f),numJoints); //should p0, decay and p1 be fractions of the remaining time (after attack)?
		setParams("decay",FloatParam(0.25f,0,0.8f),numJoints);

	}

};


//////////////////////////////////////////////////////////////////////////

class X2Machine 
{
public:
	X2Machine(X2Params* params=NULL);
	void update(float simulationTime);

protected:
	Part* createCylinderCore(NxActor* attachTo, NxMat34& transform);
	Part* createArm(NxActor* attachTo, NxMat34& transform);
	Part* createBase(NxActor* attachTo, NxMat34& transform);
	Part* createTip(NxActor* attachTo, NxMat34& transform);
	void initMeshes();

	float m_size;
	//Part** m_robParts;
	std::vector<Part*> m_robParts;
	std::vector<NxRevoluteJoint*> m_robJoints; //only the revolute motors
	MeshGraphicsObject* m_outerRing;
	MeshGraphicsObject* m_arm;
	MeshGraphicsObject* m_base;
	MeshGraphicsObject* m_tip;
	X2Params *m_params;

};


#endif
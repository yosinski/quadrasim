#ifndef X2INDUSTRIAL_H
#define X2INDUSTRIAL_H

#include "x2machine.h"

//make a more advanced x2-based machine here, inheriting x2machine
//crawler with more legs or similar
struct X2IndustrialParams : public FloatParamMultiValList
{
	X2IndustrialParams() {

		//deduce initAngle and limitAmp from limLow and limHigh, easier for control system
		//but keep limlow/high as parameters because easiest to specify joint min/max

		//int numJoints=3;
		int numJoints=2; //for now: disable first joint

		//setParams("limLow",FloatParam(0.5f,0,1,6),numJoints);
		//paramList["limLow"][1]=FloatParam(0.5f,0,0.8f,6); //hardcode way of changing a param in the list
		//alternatively: use addparam repetitively
		//addParam("limLow",FloatParam(0.1f,0.001f,0.01f,4));
		//addParam("limLow",FloatParam(0.6f,0.001f,0.7f,8));
		addParam("limLow",FloatParam(0.6f,0.3f,0.55f,8));
		addParam("limLow",FloatParam(0.6f,0.2f,0.55f,8));

		//setParams("limHigh",FloatParam(0.5f,0,1,6),numJoints);
		//addParam("limHigh",FloatParam(0.1f,0.001f,0.01f,4));
		//addParam("limHigh",FloatParam(0.01f,0.001f,0.1f,4));
		addParam("limHigh",FloatParam(0.01f,-0.29f,0.01f,4));
		//addParam("limHigh",FloatParam(0.1f,0.001f,0.1f,4));
		addParam("limHigh",FloatParam(0.1f,-0.19f,0.1f,4));


		//addParam("freq",FloatParam(0.15f,0.05,0.051f,3));
		//addParam("freq",FloatParam(0.15f,0.1f,0.11f,3));
		addParam("freq",FloatParam(0.05f,0.1f,0.11f,3));


		//setParams("phase",FloatParam(0.0f,0,1),numJoints);
		//addParam("phase",FloatParam(0.0f,0,1)); //j1
		addParam("phase",FloatParam(0.3f,0,1)); //j2
		addParam("phase",FloatParam(0.5f,0,1)); //j3

		//setParams("attack",FloatParam(0.25f,0,0.8f),numJoints); //not sure how to fix this so that all combinations will be smooth
		//setParams("p0",FloatParam(0.25f,0,0.8f),numJoints); //should p0, decay and p1 be fractions of the remaining time (after attack)?
		//setParams("decay",FloatParam(0.25f,0,0.8f),numJoints);

		//now: attack, p0, decay, p1 will be normalized 
		setParams("attack",FloatParam(0.49f,0.4f,1.0f),numJoints); 
		setParams("p0",FloatParam(0.01f,0,0.6f),numJoints); 
		setParams("decay",FloatParam(0.49f,0.4f,1.0f),numJoints);
		setParams("p1",FloatParam(0.2f,0.0f,0.6f),numJoints);

	}

};


//////////////////////////////////////////////////////////////////////////

class X2Industrial : public X2Machine
{
public:
	X2Industrial(X2IndustrialParams* params=NULL);
	//void update(float simulationTime);
	//NxVec3 getPosition(void);

protected:

private:
};

///////////////////////////////////////////////////////////

class X2IndustrialEvolutionHarness : public EvolutionHarness
{
public:
	X2IndustrialEvolutionHarness(int seed);
	~X2IndustrialEvolutionHarness();
};





#endif

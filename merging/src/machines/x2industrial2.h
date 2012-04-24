#ifndef X2INDUSTRIAL2_H
#define X2INDUSTRIAL2_H

#include "x2machine.h"

//make a more advanced x2-based machine here, inheriting x2machine
//crawler with more legs or similar
struct X2Industrial2Params : public FloatParamMultiValList
{
	X2Industrial2Params() {

		//deduce initAngle and limitAmp from limLow and limHigh, easier for control system
		//but keep limlow/high as parameters because easiest to specify joint min/max

		int numJoints=6;

		setParams("limLow",FloatParam(0.5f,0,1,6),numJoints);

		setParams("limHigh",FloatParam(0.5f,0,1,6),numJoints);

		addParam("freq",FloatParam(0.05f,0.1f,0.11f,3));

		setParams("phase",FloatParam(0.0f,0,1),numJoints);

		//now: attack, p0, decay, p1 will be normalized 
		setParams("attack",FloatParam(0.49f,0.4f,1.0f),numJoints); 
		setParams("p0",FloatParam(0.01f,0,0.6f),numJoints); 
		setParams("decay",FloatParam(0.49f,0.4f,1.0f),numJoints);
		setParams("p1",FloatParam(0.2f,0.0f,0.6f),numJoints);

	}

};


//////////////////////////////////////////////////////////////////////////

class X2Industrial2 : public X2Machine
{
public:
	X2Industrial2(X2Industrial2Params* params=NULL);

protected:

private:
};

///////////////////////////////////////////////////////////

class X2Industrial2EvolutionHarness : public EvolutionHarness
{
public:
	X2Industrial2EvolutionHarness(int seed);
	~X2Industrial2EvolutionHarness();
};





#endif

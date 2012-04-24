#ifndef X2CRAWLER_H
#define X2CRAWLER_H

#include "x2machine.h"


//make a more advanced x2-based machine here, inheriting x2machine
//crawler with more legs or similar
struct X2CrawlerParams : public FloatParamMultiValList
{
	X2CrawlerParams::X2CrawlerParams() {

		//deduce initAngle and limitAmp from limLow and limHigh, easier for control system
		//but keep limlow/high as parameters because easiest to specify joint min/max
		addParam("freq",FloatParam(0.03f,0.04f,0.042f,3));
		
		addParam("tipLength",FloatParam(X2_TIPLEN,X2_TIPLEN,17));
		addParam("armLength",FloatParam(X2_ARMLEN,X2_ARMLEN,18.0f));

		//int numJoints=12;
		int numJoints=8; //disabled first joint

		setParams("limLow",FloatParam(0.2f,-0.1f,0.1f,6),numJoints); //dette var bra
		//setParams("limLow",FloatParam(0.2f,-0.2f,0.3f,6),numJoints);
		setParams("limHigh",FloatParam(0.2f,0.11f,0.55f,7),numJoints); //dette var bra
		//setParams("limHigh",FloatParam(0.2f,0.21f,0.6f,7),numJoints);

		setParams("phase",FloatParam(0.0f,0,1),numJoints);  //kanskje ha noen faseregler?

		//now: attack, p0, decay, p1 will be normalized 
		setParams("attack",FloatParam(0.49f,0.4f,1.0f,6),numJoints); 
		setParams("p0",FloatParam(0.1f,0,0.6f,6),numJoints); 
		setParams("decay",FloatParam(0.49f,0.4f,1.0f,6),numJoints);
		setParams("p1",FloatParam(0.2f,0.0f,0.6f,6),numJoints);

		//husk å ta vare på det asymmetriske slik at man kan komme tilbake til og kontrollere på denne måten igjen
		//params + control func
	}

};


//////////////////////////////////////////////////////////////////////////

class X2Crawler : public X2Machine
{
public:
	X2Crawler(X2CrawlerParams* params=NULL,bool applyLegLength=false);
	//void update(float simulationTime);
	//NxVec3 getPosition(void);
	bool m_touchSensors[3];

protected:

	Part* create3Axis(Part* attachTo,NxMat34 tfm,int &partNo);

private:
	float m_tipLength;
	float m_armLength;

};


///////////////////////////////////////////////////////////

class X2CrawlerEvolutionHarness : public EvolutionHarness
{
public:
	X2CrawlerEvolutionHarness(int seed);
	~X2CrawlerEvolutionHarness();
};



#endif
#ifndef EVOLEG_H
#define EVOLEG_H

//hva med:
//evolver class {
// -Machine *machine
// -virtual setup()
// -evolve()


//machine class {
// -virtual create()
// -virtual update()


#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include "physx/MyCloth.h"
#include "physics.h"


struct LegParams {
	//for muscle0, muscle1:
	//radius, height, attach_high,attach_lo
	//the volumes, thus radius and height, need to be equal (antagonist muscles)

	//forandre på disse til forenklet (droppe festepunkter på benet men tenke platform, som på tavle)
	int nParams;
	float *minVals;
	float *maxVals;
	//std::pair<float,float> *ranges;
	//could maybe have a hash map or something for ease of use.. index by var name
	//gene struct {val,min,max}

	float width;
	float muscleRadius;
	float muscleHeight;
	float m0ath;
	float m0atl;
	float m1ath;
	float m1atl;
	//lower leg: 
	float l0height;
	float l0angle;
	//control!
	//    ___
	//   /   \___
	//     p0  p1
	// up and down speed dependent on muscle vol
	float pause0;
	float pause1;

	//static const float rangeMin[] = {0.5f,   4,  6};
	//static const float rangeMax[] = {3.0f,  20, 80};
	void print()
	{
		printf("width: %f mRad: %f  mHeight: %f\n",width,muscleRadius,muscleHeight);
		printf("m0ath: %f m0atl: %f m1ath: %f m1atl: %f\n",m0ath,m0atl,m1ath,m1atl);
		//add more..
	}
	
	LegParams::LegParams() {
		nParams=11;
		minVals=new float[nParams];
		maxVals=new float[nParams];

		//SIMPLIFY!!!

		width=6.6359f;
		minVals[0]=4.5f; maxVals[0]=10.0f;

		muscleRadius=3.767f;
		minVals[1]=0.5f; maxVals[1]=4.0f;

		muscleHeight=13.15f;
		minVals[2]=2.0f; maxVals[2]=20.0f;

		m0ath=0.88f;
		minVals[3]=0.1f; maxVals[3]=1.0f;

		m0atl=0.566f;
		minVals[4]=0.1f; maxVals[4]=1.0f;

		m1ath=0.379f;
		minVals[5]=0.1f; maxVals[5]=1.0f;

		m1atl=0.894f;
		minVals[6]=0.1f; maxVals[6]=1.0f;

		l0height=15.18f;
		minVals[7]=1.0f; maxVals[7]=16.0f;

		l0angle=-0.03f;
		minVals[8]=-0.6f; maxVals[8]=0.6f;

		pause0=0.5f;
		minVals[9]=0.0f; maxVals[9]=4.0f;

		pause1=0.5f;
		minVals[10]=0.0f; maxVals[10]=4.0f;

	}

};

//finn ut av hva ting egentlig burde hete her!

class LegEvolutionHarness
{
public:
	//void initEvolutionaryRun(int seed);
	LegEvolutionHarness(int seed);
	~LegEvolutionHarness();
	void evolve();
	void printStatistics();

	bool isRunning() { return !stopEvolution; }
	void stop() { stopEvolution=true; }

	void createLeg(LegParams& params);

	//rename these
	void updateLeg(float progress);
	void releaseLeg();

	MyCloth* mus1;
	MyCloth* mus2;
	Part* topPart;

private:
	MyCloth* createLegMuscle(NxVec3& pos, float height,float radius,int heightDivs,int aroundDivs);

	GASimpleGA *ga;
	bool stopEvolution;
};


#endif
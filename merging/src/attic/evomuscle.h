#ifndef EVOMUSCLE_H
#define EVOMUSCLE_H

#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include "physx/MyCloth.h"
#include "physics.h"

struct MuscleParams {
	float height;
	float amplitude;
	int heightDivs;
	int aroundDivs;
};

//finn ut av hva ting egentlig burde hete her!

class MuscleEvolutionHarness
{
public:
	//void initEvolutionaryRun(int seed);
	MuscleEvolutionHarness(int seed);
	~MuscleEvolutionHarness();
	void evolve();
	void printStatistics();

	bool isRunning() { return !stopEvolution; }
	void stop() { stopEvolution=true; }

	void setupMuscle(MuscleParams& params);
	void updateMuscle(float progress);
	//void updateMuscle(unsigned int frame);
	void releaseMuscle();

	MyCloth* evoMuscle;
	Part* hangingBox;
	NxVec3 attachTopPos;

private:
	GASimpleGA *ga;
	bool stopEvolution;
};


#endif
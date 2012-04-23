#ifndef EVOLUTIONHARNESS_H
#define EVOLUTIONHARNESS_H

#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include "base/floatparammultivallist.h"


struct GenerationElite {
	int generation;
	float fitness;
	//params
};



class EvolutionHarness
{
public:
	EvolutionHarness();
	~EvolutionHarness();

	virtual void evolve(bool continueFromFile=false);
	virtual void printStatistics(char *fileName="bestind.txt");
	bool isRunning() { return !stopEvolution; }
	virtual void stop() { stopEvolution=true; }
	virtual void updateGraphicsAndPhysics(bool freeze=false);
	void setSimulatorTimeStep(double timeStep);
	int generationNumber();
	double m_timestep;
	void convertParams( GAGenome& g, FloatParamMultiValList &p );

protected:
	GASimpleGA *m_ga;
	bool stopEvolution;

	void createGA(const FloatParamMultiValList& p, GAGenome::Evaluator evaluator, const char* paramFileName);

};

void forcedPopEvaluator(GAPopulation & p);


#endif
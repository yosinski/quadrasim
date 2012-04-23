#ifndef EVODOG_H
#define EVODOG_H

#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include "physx/MyCloth.h"
#include "physics.h"
#include "evolutionharness.h"

class DogEvolutionHarness : public EvolutionHarness
{
public:
	DogEvolutionHarness(int seed);
	~DogEvolutionHarness();

};

#endif
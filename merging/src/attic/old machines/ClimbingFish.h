#pragma once

#include <vector>
using namespace std;

#include "NxPhysics.h"

#include "../base/floatparammultivallist.h"
#include "../EvolutionHarness.h"


struct ClimbingFishParams : public FloatParamMultiValList {
	ClimbingFishParams() {
		int numberOfActuators = 12;
		setParams("actuatorAttackPhase", FloatParam(0, 0, 1, 6), numberOfActuators); 
		setParams("actuatorReleasePhase", FloatParam(.5, 0, 1, 6), numberOfActuators); 
	}
};


struct Body;
struct TripartiteLimb;

struct ClimbingFish {
	ClimbingFish(ClimbingFishParams& params);
	~ClimbingFish();

	NxVec3 getPosition();

	void update(float simulationTime);

	Body* body;
	vector<TripartiteLimb*> limbs;
};


class ClimbingFishEvolutionHarness : public EvolutionHarness {
public:
	ClimbingFishEvolutionHarness(int seed);
};

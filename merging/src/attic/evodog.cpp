#include "evodog.h"
#include "machines/dogmachine.h"
#include "physics.h"
#include "graphics/graphics.h"
#include "base/system.h"
#include "physx/MyCloth.h"
#include "graphics/glstuff.h"
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#define cout STD_COUT

//todo: vurder om dette også burde vært inkludert i dogmachine for å få færre filer

float calculateDogFitness(GAGenome& g)
{
	DogEvolutionHarness* evoHarness = (DogEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;
	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	//set up dog from genome
	DogParams p;
	GABin2DecGenome& genome = (GABin2DecGenome &)g;
	std::map<std::string, FloatParam>::iterator it;
	int i=0;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		it->second.val=genome.phenotype(i);
		i++;
	}
	//p.print(); //for debugging
	/*GABin2DecGenome* ind=(GABin2DecGenome*)&g;
	ind->GA1DBinaryStringGenome::write(cout);
	cout << std::endl;*/

	DogMachine machine(p);

	NxVec3 firstPos=machine.topPart->physicsEquivalent->getGlobalPose().t;
	NxVec3 lastPos=firstPos;
	float highestPos=firstPos.y;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;
	int runFrames=600; 
	for(int f=0;f<runFrames;f++) {
		bool freeze=false;
		if(glfwGetKey(GLFW_KEY_F3)) freeze=true;
		if( glfwGetKey(GLFW_KEY_ESC) ) {
			evoHarness->stop();
			break; 
		}

		if(freeze && f>0) //litt dumt kanskje ha while i stedet
			f--;
		machine.update((float)simTime);
		setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0);
		evoHarness->updateGraphicsAndPhysics(freeze,(float)evoHarness->timestep);
		if(!freeze)
			simTime+=evoHarness->timestep;

		lastPos=machine.topPart->physicsEquivalent->getGlobalPose().t;

		//try to detect jumping/floating
		float y=machine.topPart->physicsEquivalent->getGlobalPose().t.y;
		if(y > firstPos.y*1.2f || y < firstPos.y*0.5f) {
			cheat=true;
			break;
		}
		//try also to detect hovering in some way
	}
	NxVec3 diff=machine.topPart->physicsEquivalent->getGlobalPose().t-firstPos;
	terminatePhysics();

	float fitnessValue=diff.magnitude();
	if(cheat) {
		printf("cheat!\n");
		fitnessValue=0;
	}

	printf("fit: %f\n",fitnessValue);
	return fitnessValue;
}


//her kan det kanskje flyttes mer ut til superklasse?
//sjekk hvordan det kan fikses
DogEvolutionHarness::DogEvolutionHarness(int seed)
	: EvolutionHarness()
{
	//ga=NULL;
	//stopEvolution=false;
	GARandomSeed(seed);
	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	char *paramFile="settings.txt"; //read parameters from file, overwrite default settings!
	params.read(paramFile); 

	//mapping of bits to decimals
	GABin2DecPhenotype mapping;
	DogParams p; //initializes min/max values
	//må ha iterator..
	std::map<std::string, FloatParam>::iterator it;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		mapping.add(8, it->second.minVal, it->second.maxVal); //8b precision
	}

	GABin2DecGenome genome(mapping, calculateDogFitness, (void *)this); //pass a pointer to this object
	m_ga = new GASimpleGA(genome);
	if(NULL==m_ga)
		systemError("could not alloc GA");
	m_ga->parameters(params);
	stopEvolution=false;
}

DogEvolutionHarness::~DogEvolutionHarness()
{
	if(gPhysicsSDK)
		terminatePhysics();
}

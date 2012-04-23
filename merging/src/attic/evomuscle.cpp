#include "evomuscle.h"

#include "physics.h"
#include "graphics/graphics.h"
#include "testobjects.h"
#include "base/system.h"
#include "physx/MyCloth.h"
#include "graphics/glstuff.h"
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#define cout STD_COUT

//burde lage egen createMuscle her så ikke avhengig av testobjects
//se evoleg

//ha bit precision i tankene
static const unsigned int nGenes=3;
static const float rangeMin[] = {0.5f,   4,  6};
static const float rangeMax[] = {20.0f,  20, 80};
//found 3 12 77

float calculateMuscleFitness(GAGenome& g)
{
	MuscleEvolutionHarness* evoHarness = (MuscleEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;

	if(gPhysicsSDK) 
		terminatePhysics();

	initPhysics();

	MuscleParams p;
	GABin2DecGenome& genome = (GABin2DecGenome &)g;

	p.height=10; //we must have constant height?
	p.amplitude=genome.phenotype(0);
	p.heightDivs=(int)genome.phenotype(1);
	p.aroundDivs=(int)genome.phenotype(2);

	//decode genome here, set params
	evoHarness->setupMuscle(p);

	NxVec3 attachBotPos;
	float maxStretch=p.height;
	float minStretch=p.height;
	int runFrames=200; 
	static bool visualize = true;
	for(int f=0;f<runFrames;f++) {
		if(glfwGetKey(GLFW_KEY_F1)) visualize=!visualize;
		updateFPS();
		updateWindowSize();

		//map progress function
		//have stabilizing time in beginning and end
		float progress=((float)(f*1.2)/runFrames)-0.1f;
		progress=__min(1,__max(progress,0));
		//printf("%.2f ",progress);

		evoHarness->updateMuscle(progress);
		gScene->simulate(1.0f/60.0f);
		
		//graphics in the meantime..
		if(visualize) 
			updateGraphics();

		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		//measure stretch and save if large
		attachBotPos=evoHarness->hangingBox->physicsEquivalent->getGlobalPosition();
		float stretch=attachBotPos.distance(evoHarness->attachTopPos);
		/*if(stretch>maxStretch) {
			maxStretch=stretch;
		}*/
		if(stretch<minStretch) {
			minStretch=stretch;
		}

		if(visualize) {
			if(glfwGetKey(GLFW_KEY_LCTRL)) 
				renderDebugInfo(); //hvorfor kan ikke dette også rendres før fetchresults?

			glfwSwapBuffers(); //screen update  
		}
		else
			glfwPollEvents(); //this must be called if swapbuffers is not called

		if( glfwGetKey(GLFW_KEY_ESC) ) {
			evoHarness->stop();
			break;
		}
	}

	evoHarness->releaseMuscle();
	terminatePhysics();

	float fitnessValue=__max(maxStretch-minStretch, 0);
	printf("fit: %f\n",fitnessValue);
	return fitnessValue;
}

MuscleEvolutionHarness::~MuscleEvolutionHarness()
{
	//NOT TESTED!
	releaseMuscle();
	if(gPhysicsSDK)
		terminatePhysics();
	if(ga)
		delete ga;
}


//void MuscleEvolutionHarness::initEvolutionaryRun(int seed)
MuscleEvolutionHarness::MuscleEvolutionHarness(int seed)
{
	ga=NULL;
	stopEvolution=false;
	evoMuscle=NULL;
	hangingBox=NULL;
	attachTopPos.set(0,0,0);

	GARandomSeed(seed);

	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	//fjern disse etterhvert
	params.set(gaNnGenerations, 100);
	params.set(gaNpopulationSize, 30);
	params.set(gaNpCrossover, 0.9);
	params.set(gaNpMutation, 0.001);
	params.set(gaNscoreFilename, "bog.dat");
	params.set(gaNscoreFrequency, 10);
	params.set(gaNflushFrequency, 50);

	//read parameters from file, overwrite default settings!
	char *paramFile="muscle_settings.txt";
	params.read(paramFile); 

	//mapping of bits to decimals
	GABin2DecPhenotype map;
	for(unsigned int i=0; i<nGenes; i++)
		map.add(8, rangeMin[i], rangeMax[i]); //8b precision

	//GABin2DecGenome genome(map, calculateMuscleFitness, (void *)target);
	GABin2DecGenome genome(map, calculateMuscleFitness, (void *)this); //pass a pointer to this object
	ga = new GASimpleGA(genome);
	ga->parameters(params);

	stopEvolution=false;
}


//void MuscleEvolutionHarness::updateMuscle(unsigned int frame)
void MuscleEvolutionHarness::updateMuscle(float progress)
{
	if(!evoMuscle) return;

	//verify function
	float maxPressure=2.0f;
	//float p=(sin(progress*PI/2.0)*0.5f+0.5f);
	float p=(sin(progress*PI/2.0f));
	evoMuscle->getNxCloth()->setPressure(p*maxPressure+1); //1-(1+maxpressure)
}


void MuscleEvolutionHarness::setupMuscle(MuscleParams& params)
{
	//float height=10;
	//int heightDivs=8;
	//int aroundDivs=16;
	float cubeSize=0.6f;
	//float startHeight=10.0f;
	float startHeight=cubeSize*2;
	int endPoint=(params.heightDivs+1)*params.aroundDivs-1;
	float xOffs=0;
	//Part* cube;

	evoMuscle=generateMuscle(NxVec3(xOffs,startHeight,0),params.height,params.amplitude,params.heightDivs,params.aroundDivs);
	evoMuscle->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+params.height,0));
	hangingBox=createBoxPart(Point3D(xOffs,startHeight-cubeSize,0),Point3D(cubeSize,cubeSize,cubeSize));
	(hangingBox);
	//hangingBox->physicsEquivalent->setMass(0.6f); //for tung last kan ødelegge..
	hangingBox->physicsEquivalent->setMass(0.8f); //for tung last kan ødelegge..
	evoMuscle->getNxCloth()->attachToShape(hangingBox->physicsEquivalent->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);

	attachTopPos=evoMuscle->getNxCloth()->getVertexAttachmentPosition(endPoint);

}

void MuscleEvolutionHarness::releaseMuscle()
{
	evoMuscle=NULL;
	hangingBox=NULL;
	attachTopPos.set(0,0,0);
}

void MuscleEvolutionHarness::evolve()
{
	stopEvolution=false;
	double startTime=glfwGetTime();
	ga->initialize(); //this apparently evaluates all individuals
	printf("generation time: %f\n",glfwGetTime()-startTime);
	while(!ga->done() && !stopEvolution){
		printf("gen: %d\n",ga->statistics().generation());
		ga->step();
	}
	ga->flushScores();
}

void MuscleEvolutionHarness::printStatistics()
{
	cout << "The GA found:\n" << ga->statistics().bestIndividual() << "\n" << "fitness: " << ga->statistics().bestIndividual().score() << "\n";
	cout << "seed:" << GAGetRandomSeed() << "\n";
}
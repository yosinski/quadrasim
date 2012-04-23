#include "evoleg.h"
#include "physics.h"
#include "graphics.h"
#include "system.h"
#include "physx/MyCloth.h"
#include "glstuff.h"
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#define cout STD_COUT


//TODO: fix control system. Make frame/simulation time dependent not #frames dependent

MyCloth* LegEvolutionHarness::createLegMuscle(NxVec3& pos, float height,float radius,int heightDivs,int aroundDivs)
{
	NxClothDesc clothDesc;
	clothDesc.globalPose.t = pos;
	//clothDesc.dampingCoefficient = 0.8f; //hva er mye? 1 eller 0?
	clothDesc.density=1.0f;
	clothDesc.friction = 0.5f;
	clothDesc.pressure = 1.0f;
	clothDesc.solverIterations = 8; //default: 5
	clothDesc.flags |= NX_CLF_PRESSURE;
	clothDesc.flags |= NX_CLF_GRAVITY;
	clothDesc.flags |= NX_CLF_BENDING;
	clothDesc.flags |= NX_CLF_BENDING_ORTHO; //more realistic
	clothDesc.flags |= NX_CLF_DAMPING;
	clothDesc.flags |= NX_CLF_COLLISION_TWOWAY; //cloth can also pull other things
	if(gHardwareSimulation)
		clothDesc.flags |= NX_CLF_HARDWARE;
	MyCloth *objCloth = new MyCloth(gScene,clothDesc,height,radius,heightDivs,aroundDivs); //use handmade muscle constructor
	cloths.push_back(objCloth);
	return objCloth;
}


void LegEvolutionHarness::createLeg(LegParams& p)
{
	float x,y,z;
	MyCloth* legMus1=NULL;
	MyCloth* legMus2=NULL;
	x=y=z=0;

	z=100.0f;
	float w=p.width; //half width of setup
	float density=1.5f;
	//Part* topPart=createBoxPart(Point3D(0,p.l0height+p.muscleHeight+3,z),Point3D(w+1,1,1));
	topPart=createBoxPart(Point3D(0,p.l0height+p.muscleHeight+3,z),Point3D(w+1,1,1));
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions=NxVec3(1,p.muscleHeight/2-1,1);
	boxDesc.localPose.t.set(0,-p.muscleHeight/2,0);
	topPart->physicsEquivalent->createShape(boxDesc);
	topPart->physicsEquivalent->updateMassFromShapes(density,0); //constant density

	Part* botPart=createBoxPart(Point3D(0,p.l0height+1,z),Point3D(w+1,1,1));
	boxDesc.dimensions=NxVec3(1,p.l0height/2,1);
	boxDesc.localPose.t.set(0,-p.l0height/2-1,0);
	boxDesc.localPose.M.rotZ(p.l0angle);
	botPart->physicsEquivalent->createShape(boxDesc);
	botPart->physicsEquivalent->updateMassFromShapes(density,0); //constant density

	//top-bot joint
	NxRevoluteJoint* j=createRevoluteJoint(topPart->physicsEquivalent,botPart->physicsEquivalent,NxVec3(0,p.l0height+2.5f,z),NxVec3(0,0,1));

	/*
	//harness and joints
	Part* radialRod=createBoxPart(Point3D(0,p.l0height+p.muscleHeight+3,z/2-1.0f),Point3D(0.4f,0.4f,z/2-2.0f));
	//radialRod->physicsEquivalent->updateMassFromShapes(0.2f,0);
	createRevoluteJoint(topPart->physicsEquivalent,radialRod->physicsEquivalent,NxVec3(0,p.l0height+p.muscleHeight+3,z-0.5f),NxVec3(1,0,0));

	Part* radialRod2=createBoxPart(Point3D(0,p.l0height+p.muscleHeight/2+3,z/2-1.0f),Point3D(0.4f,0.4f,z/2-2.0f));
	//radialRod2->physicsEquivalent->updateMassFromShapes(0.2f,0);
	createRevoluteJoint(topPart->physicsEquivalent,radialRod2->physicsEquivalent,NxVec3(0,p.l0height+p.muscleHeight/2+3,z-0.5f),NxVec3(1,0,0));

	Part* centerRod=createBoxPart(Point3D(0,(p.l0height+p.muscleHeight+3)/2,0),Point3D(0.5f,(p.l0height+p.muscleHeight+3)/2,0.5f));
	centerRod->physicsEquivalent->updateMassFromShapes(2.0f,0);
	createRevoluteJoint(radialRod->physicsEquivalent,centerRod->physicsEquivalent,NxVec3(0,p.l0height+p.muscleHeight+3,0),NxVec3(1,0,0));
	createRevoluteJoint(radialRod2->physicsEquivalent,centerRod->physicsEquivalent,NxVec3(0,p.l0height+p.muscleHeight/2+3,0),NxVec3(1,0,0));
	createRevoluteJoint(centerRod->physicsEquivalent,NULL,NxVec3(0,0,0),NxVec3(0,1,0));
	*/

	int heightDivs=8;
	int aroundDivs=40;
	int endPoint=(heightDivs+1)*aroundDivs-1;
	float attachHigh=p.l0height+p.muscleHeight+2.0f;
	float attachLow=p.l0height+2.0f; //rod thickness

	//mus1=createLegMuscle(NxVec3(p.m0atl*w+1,attachLow,z),p.muscleHeight,p.muscleRadius,heightDivs,aroundDivs);
	//try creating in middle and attaching at correct point afterwards in order to avoid collision with middle part
	mus1=createLegMuscle(NxVec3(0.5f*w+1,attachLow,z),p.muscleHeight,p.muscleRadius,heightDivs,aroundDivs);
	mus1->getNxCloth()->attachVertexToShape(0,botPart->physicsEquivalent->getShapes()[0],NxVec3(p.m0atl*w+1,1,0),NX_CLOTH_ATTACHMENT_TWOWAY);
	mus1->getNxCloth()->attachVertexToShape(endPoint,topPart->physicsEquivalent->getShapes()[0],NxVec3(p.m0ath*w+1,-1,0),NX_CLOTH_ATTACHMENT_TWOWAY);

	//mus2=createLegMuscle(NxVec3(-p.m1atl*w-1,attachLow,z),p.muscleHeight,p.muscleRadius,heightDivs,aroundDivs);
	mus2=createLegMuscle(NxVec3(-0.5f*w-1,attachLow,z),p.muscleHeight,p.muscleRadius,heightDivs,aroundDivs);
	mus2->getNxCloth()->attachVertexToShape(0,botPart->physicsEquivalent->getShapes()[0],NxVec3(-p.m1atl*w-1,1,0),NX_CLOTH_ATTACHMENT_TWOWAY);
	mus2->getNxCloth()->attachVertexToShape(endPoint,topPart->physicsEquivalent->getShapes()[0],NxVec3(-p.m1ath*w-1,-1,0),NX_CLOTH_ATTACHMENT_TWOWAY);
	
}

void LegEvolutionHarness::updateLeg(float progress)
{
	if(!mus1 || !mus2) return;

	//not really period count any more
	float periods=3.0f;

	//float maxPressure=3.0f;
	float maxPressure=2.0f;
	float pressure=sin(periods*progress*2.0f*PI)*0.5f+0.5f;
	mus1->getNxCloth()->setPressure(pressure*maxPressure+1.2f);
	mus2->getNxCloth()->setPressure((1.0f-pressure)*maxPressure+1.2f);
}



float calculateLegFitness(GAGenome& g)
{
	
	LegEvolutionHarness* evoHarness = (LegEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;

	if(gPhysicsSDK) 
		terminatePhysics();

	initPhysics();

	LegParams p;
	GABin2DecGenome& genome = (GABin2DecGenome &)g;

	//MAKE RESET FUNCTIONALITY FOR EACH INDIVIDUAL!!

	//this mapping should be simplified with a better LegParams struct
	p.width=genome.phenotype(0);
	p.muscleRadius=genome.phenotype(1);
	p.muscleHeight=genome.phenotype(2);
	p.m0ath=genome.phenotype(3);
	p.m0atl=genome.phenotype(4);
	p.m1ath=genome.phenotype(5);
	p.m1atl=genome.phenotype(6);
	p.l0height=genome.phenotype(7);
	p.l0angle=genome.phenotype(8);
	p.pause0=genome.phenotype(9);
	p.pause1=genome.phenotype(10);
	
	p.print(); //for debugging

	//decode genome here, set params
	evoHarness->createLeg(p);

	//UPDATE EVALUATION!!!
	//ALSO NOW NOT CORRECT CONTROL MECHANISMS!!
	

	NxVec3 firstPos=evoHarness->topPart->physicsEquivalent->getGlobalPose().t;
	NxVec3 lastPos=firstPos;
	float distSum=0;
	float highestPos=lastPos.y;

	int runFrames=1000; 
	static bool visualize = true;
	for(int f=0;f<runFrames;f++) {
		bool freeze=false;
		if(glfwGetKey(GLFW_KEY_F1)) visualize=!visualize;
		if(glfwGetKey(GLFW_KEY_F3)) freeze=true;
		updateFPS();
		updateWindowSize();

		float progress=( ((float)f)/runFrames ); 
		evoHarness->updateLeg(progress);

		if(freeze) {
			if(f>0) f--;
		}
		else {
			gScene->simulate(1.0f/60.0f);
		}
		
		//graphics in the meantime..
		if(visualize) {
			updateGraphics();
			if(ENABLE_POSTPROCESSING) {
				fbo->bind();
				updateGraphics();
				fbo->unBind();
				glow(fbo,0.7f);
			}
		}

		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		//add distance difference for every frame
		NxVec3 curPos=evoHarness->topPart->physicsEquivalent->getGlobalPose().t;
		if(curPos.y>highestPos)
			highestPos=curPos.y;
		
		NxVec3 up(0,1,0);
		NxVec3 forward=up.cross(curPos);
		forward.normalize();
		NxVec3 delta=curPos-lastPos;
		float deltaLength=delta.dot(forward);
		distSum+=deltaLength;
		lastPos=curPos;

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

	evoHarness->releaseLeg();
	terminatePhysics();


	float fitnessValue=distSum;
	
	//try to kill singularities
	if(highestPos > firstPos.y*2 )
		fitnessValue=0;
	
	printf("fit: %f\n",fitnessValue);
	return fitnessValue;

	return 0;
}

LegEvolutionHarness::~LegEvolutionHarness()
{
	//NOT TESTED!
	releaseLeg();
	if(gPhysicsSDK)
		terminatePhysics();
	if(ga)
		delete ga;
}


//void LegEvolutionHarness::initEvolutionaryRun(int seed)
LegEvolutionHarness::LegEvolutionHarness(int seed)
{
	ga=NULL;
	stopEvolution=false;
	mus1=mus2=NULL;

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
	char *paramFile="leg_settings.txt";
	params.read(paramFile); 

	//mapping of bits to decimals
	GABin2DecPhenotype map;
	LegParams p; //initializes min/max values
	for(int i=0; i<p.nParams; i++)
		map.add(8, p.minVals[i], p.maxVals[i]); //8b precision

	//GABin2DecGenome genome(map, calculateMuscleFitness, (void *)target);
	GABin2DecGenome genome(map, calculateLegFitness, (void *)this); //pass a pointer to this object
	ga = new GASimpleGA(genome);
	ga->parameters(params);

	stopEvolution=false;
}





void LegEvolutionHarness::releaseLeg()
{
	mus1=mus2=NULL;
	topPart=NULL;
}

void LegEvolutionHarness::evolve()
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

void LegEvolutionHarness::printStatistics()
{
	cout << "The GA found:\n" << ga->statistics().bestIndividual() << "\n" << "fitness: " << ga->statistics().bestIndividual().score() << "\n";
	cout << "seed:" << GAGetRandomSeed() << "\n";
}
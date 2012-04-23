#include "x2industrial.h"

#include <assert.h>
#include "../physics.h"
#include "../base/system.h"
#include "../graphics/graphics.h"


X2Industrial::X2Industrial(X2IndustrialParams* params/* =NULL */)
{
	if(NULL==params)
			params=new X2IndustrialParams();
	assert(params);
	//params->print();
	m_params=params;
	//FIX: need to resolve if params should be pointer or someth else. does it copy with maps and vectors automatically?

	Part* lastPart=NULL;
	Part* curPart=NULL;
	int jointNo=0;

	lastPart=createPlatform(NULL,createTfm(0,0,0),false); //platform (true fixes to ground)
	lastPart=createBase(lastPart->act,createTfm(0,2.5f,0),true); //base

	lastPart=createCylinderCore(lastPart->act,createTfm(0,11.3f,0),true); //core 1
	createLid(lastPart->act,createTfm(0,0,-m_lidOffset,0,PI/2,0),true); //lid core 1
	createLid(lastPart->act,createTfm(0,0,m_lidOffset,0,-PI/2,0),true); //lid core 1
	createEncoderBox(lastPart->act,createTfm(m_encBoxOffset,0,0,PI,0,0),true); //encoder box core 1

	//core 2 (now: fixed joint)
	//curPart=createCylinderCore(lastPart->act,createTfm(0,coreOffset,0,PI/2,0,getInitAngle(jointNo)));
	curPart=createCylinderCore(lastPart->act,createTfm(0,m_coreOffset,0,PI/2,0,0),true); //false if motor
	//joint=createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;
	createLid(lastPart->act,createTfm(0,0,-m_lidOffset,0,PI/2,0),true); //lid core 2
	createEncoderBox(lastPart->act,createTfm(m_encBoxOffset,0,0,PI,0,0),true); //encoder box core 2

	curPart=createArm(lastPart->act,createTfm(0,0,0,-PI/2,0,getInitAngle(jointNo)),false); //arm
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;

	curPart=createCylinderCore(lastPart->act,createTfm(0,23.5f,0,PI/2,getInitAngle(jointNo),0),false); //core 3
	createParamMotorJoint(curPart,NxVec3(0,-1,0),lastPart,jointNo++);
	lastPart=curPart;
	createLid(lastPart->act,createTfm(0,0,m_lidOffset,0,-PI/2,0),true); //lid core 3
	createEncoderBox(lastPart->act,createTfm(m_encBoxOffset,0,0,PI,0,0),true); //encoder box core 3

	lastPart=createGripTip(lastPart->act,createTfm(0,0,-8,-PI/2,0,0),true); //tip


	//hack to increase stability
	int solverIterations=20; //4 is default
	for each(Part* part in m_robParts) //not portable
		part->act->setSolverIterationCount(solverIterations);

}



//////////////////////////////////////////////////////////////////////////
// EVOLUTION AND FITNESS
//////////////////////////////////////////////////////////////////////////

float calculateX2IndustrialFitness(GAGenome& g)
{
	X2IndustrialEvolutionHarness* evoHarness = (X2IndustrialEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;
	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	//map from genome
	//hope this works..
	X2IndustrialParams p;
	GABin2DecGenome& genome = (GABin2DecGenome &)g;
	int i=0;
	std::map<std::string, std::vector<FloatParam> >::iterator it;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		std::vector<FloatParam>::iterator jt;
		for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
			//printf("%s: val: %.2f min: %.2f max: %.2f precision: %d\n",it->first.c_str(),jt->val,jt->minVal,jt->maxVal,jt->precision);
			jt->val=genome.phenotype(i);
			i++;
		}
	}


	X2Industrial machine(&p); //ok p not going out of scope here


	NxVec3 firstPos=machine.getPosition();
	NxVec3 lastPos=firstPos;
	double distSum=0.0;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;
	const int runFrames=3000;

	int f;
	for(f=0;f<runFrames;f++) {
		bool freeze=false;
		if(glfwGetKey(GLFW_KEY_F3)) freeze=true;
		if( glfwGetKey(GLFW_KEY_ESC) ) {
			evoHarness->stop();
			break; 
		}

		if(freeze && f>0) //litt dumt kanskje ha while i stedet
			f--;
		machine.update((float)simTime);
		//setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0);
		setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),float(0.01*simTime));
		evoHarness->updateGraphicsAndPhysics(freeze);
		if(!freeze)
			simTime+=evoHarness->m_timestep;
		lastPos=machine.getPosition();

	} //end eval loop

	NxVec3 diff=machine.getPosition()-firstPos;
	terminatePhysics();
	//int framesElapsed=f;
	//float fitnessValue=(float)__max(diff.magnitude(),0); //allows any direction of final movement, not only "forward"
	float fitnessValue=(float)__max(diff.dot(NxVec3(1,0,0)),0); //only forward
	fitnessValue=fitnessValue*60.0f/runFrames;

	if(cheat) {
		printf("cheat!");
		fitnessValue=0;
	}
	printf(" fit: %f\n",fitnessValue);
	return fitnessValue;
}


//her kan det kanskje flyttes mer ut til superklasse?
X2IndustrialEvolutionHarness::X2IndustrialEvolutionHarness(int seed)
: EvolutionHarness()
{
	GARandomSeed(seed);
	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	char *paramFile="x2settings.txt"; //read parameters from file, overwrite default settings!
	params.read(paramFile);
	//setSimulatorTimeStep(1/60.0f);
	setSimulatorTimeStep(1/120.0f);

	//initialization of mapping, bits to decimals
	int nBits=0;
	GABin2DecPhenotype mapping;
	X2IndustrialParams p; //initializes min/max values
	std::map<std::string, std::vector<FloatParam> >::iterator it;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		std::vector<FloatParam>::iterator jt;
		for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
			printf("%s: val: %.2f min: %.2f max: %.2f precision: %d\n",it->first.c_str(),jt->val,jt->minVal,jt->maxVal,jt->precision);
			mapping.add(jt->precision, jt->minVal, jt->maxVal); //specified precision
			nBits+=jt->precision;
		}
	}
	printf("total number of bits: %d\n",nBits);

	GABin2DecGenome genome(mapping, calculateX2IndustrialFitness, (void *)this); //pass a pointer to this object
	GAPopulation pop(genome);
	pop.evaluator(forcedPopEvaluator); //this uses the custom evaluator which forces the elite to be evaluated each time (?)
	m_ga = new GASimpleGA(pop);
	if(NULL==m_ga)
		systemError("could not alloc GA");
	m_ga->parameters(params);
	stopEvolution=false;
}

X2IndustrialEvolutionHarness::~X2IndustrialEvolutionHarness()
{
	if(gPhysicsSDK)
		terminatePhysics();
}



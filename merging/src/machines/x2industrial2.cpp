#include "../machines/x2industrial2.h"
#include <assert.h>
#include "../physics.h"
#include "../base/system.h"
#include "../graphics/graphics.h"
#include <math.h>
#include <boost/foreach.hpp>


X2Industrial2::X2Industrial2(X2Industrial2Params* params/* =NULL */)
{
	if(NULL==params)
			params=new X2Industrial2Params();
	assert(params);
	//params->print();
	m_params=params;
	//FIX: need to resolve if params should be pointer or someth else. does it copy with maps and vectors automatically?

	Part* lastPart=NULL;
	Part* curPart=NULL;
	int jointNo=0;

	NxMat34 temp1 = createTfm(0,0,0);
	lastPart=createBase(NULL,temp1,true); //base

	NxMat34 temp2 =createTfm(0,11.3f,0);
	lastPart=createCylinderCore(lastPart->act,temp2,true); //core 1
	NxMat34 temp3 =createTfm(0,0,-m_lidOffset,0,PI/2,0);
	createLid(lastPart->act,temp3,true); //lid core 1
	NxMat34 temp4 = createTfm(0,0,m_lidOffset,0,-PI/2,0);
	createLid(lastPart->act,temp4,true); //lid core 1
	NxMat34 temp5 =createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,temp5,true); //encoder box core 1

	//core 2 (now: fixed joint)
	NxMat34 temp6 = createTfm(0,m_coreOffset,0,PI/2,0,getInitAngle(jointNo));
	curPart=createCylinderCore(lastPart->act,temp6,false);
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;
	NxMat34 temp7 = createTfm(0,0,-m_lidOffset,0,PI/2,0);
	createLid(lastPart->act,temp7,true); //lid core 2
	NxMat34 temp8 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,temp8,true); //encoder box core 2

	NxMat34 temp9 = createTfm(0,0,0,-PI/2,0,getInitAngle(jointNo));
	curPart=createArm(lastPart->act,temp9,false); //arm
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;

	NxMat34 temp10 = createTfm(0,23.5f,0,PI/2,getInitAngle(jointNo),0);
	curPart=createCylinderCore(lastPart->act,temp10,false); //core 3
	createParamMotorJoint(curPart,NxVec3(0,-1,0),lastPart,jointNo++);
	lastPart=curPart;
	NxMat34 temp11 = createTfm(0,23.5f,0,PI/2,getInitAngle(jointNo),0);
	createLid(lastPart->act,temp11,true); //lid core 3
	NxMat34 temp12 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,temp12,true); //encoder box core 3

	//lastPart=createTip(lastPart->act,createTfm(0,0,-8,-PI/2,0,0),true); //tip


	NxMat34 temp13 = createTfm(0,0,-m_coreOffset,-PI/2,0,0);
	lastPart=createCylinderCore(lastPart->act,temp13,true); //core 1
	NxMat34 temp14 = createTfm(0,0,-m_lidOffset,0,PI/2,0);
	createLid(lastPart->act,temp14,true); //lid core 1
	NxMat34 temp15 = createTfm(0,0,m_lidOffset,0,-PI/2,0);
	createLid(lastPart->act,temp15,true); //lid core 1
	NxMat34 temp16 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,temp16,true); //encoder box core 1

	NxMat34 temp17 = createTfm(0,m_coreOffset,0,PI/2,0,getInitAngle(jointNo));
	curPart=createCylinderCore(lastPart->act,temp17,false);
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;
	NxMat34 temp18 = createTfm(0,0,-m_lidOffset,0,PI/2,0);
	createLid(lastPart->act,temp18,true); //lid core 2
	NxMat34 temp19 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,temp19,true); //encoder box core 2
	

	NxMat34 temp20 = createTfm(0,0,0,-PI/2,0,getInitAngle(jointNo));
	curPart=createArm(lastPart->act,temp20,false); //arm
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;

	NxMat34 temp21 = createTfm(0,23.5f,0,PI/2,getInitAngle(jointNo),0);
	curPart=createCylinderCore(lastPart->act,temp21,false); //core 3
	createParamMotorJoint(curPart,NxVec3(0,-1,0),lastPart,jointNo++);
	lastPart=curPart;
	NxMat34 temp22 = createTfm(0,0,m_lidOffset,0,-PI/2,0);
	createLid(lastPart->act,temp22,true); //lid core 3
	NxMat34 temp23 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,temp23,true); //encoder box core 3

	NxMat34 temp24 = createTfm(0,0,-8,-PI/2,0,0);
	lastPart=createTip(lastPart->act,temp24,true); //tip
	


	//hack to increase stability
	int solverIterations=20; //4 is default
	BOOST_FOREACH(Part* part, m_robParts) //not portable
		part->act->setSolverIterationCount(solverIterations);

}



//////////////////////////////////////////////////////////////////////////
// EVOLUTION AND FITNESS
//////////////////////////////////////////////////////////////////////////

float calculateX2Industrial2Fitness(GAGenome& g)
{
	X2Industrial2EvolutionHarness* evoHarness = (X2Industrial2EvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;
	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	//map from genome
	//hope this works..
	X2Industrial2Params p;
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


	X2Industrial2 machine(&p); //ok p not going out of scope here


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
	float fitnessValue=fmax(diff.dot(NxVec3(1,0,0)),0); //only forward
	fitnessValue=fitnessValue*60.0f/runFrames;

	if(cheat) {
		printf("cheat!");
		fitnessValue=0;
	}
	printf(" fit: %f\n",fitnessValue);
	return fitnessValue;
}


//her kan det kanskje flyttes mer ut til superklasse?
X2Industrial2EvolutionHarness::X2Industrial2EvolutionHarness(int seed)
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
	X2Industrial2Params p; //initializes min/max values
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

	GABin2DecGenome genome(mapping, calculateX2Industrial2Fitness, (void *)this); //pass a pointer to this object
	GAPopulation pop(genome);
	pop.evaluator(forcedPopEvaluator); //this uses the custom evaluator which forces the elite to be evaluated each time (?)
	m_ga = new GASimpleGA(pop);
	if(NULL==m_ga)
		systemError("could not alloc GA");
	m_ga->parameters(params);
	stopEvolution=false;
}

X2Industrial2EvolutionHarness::~X2Industrial2EvolutionHarness()
{
	if(gPhysicsSDK)
		terminatePhysics();
}



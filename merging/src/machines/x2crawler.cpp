#include "x2crawler.h"

#include <assert.h>
#include "../physics.h"
#include "../base/system.h"
#include "../graphics/graphics.h"
#include <math.h>
#include <boost/foreach.hpp>

Part* X2Crawler::create3Axis(Part* attachTo,NxMat34 tfm,int &jointNo)
{
	Part* lastPart=attachTo;
	Part* curPart=NULL;

	lastPart=createCylinderCore(lastPart->act,tfm,true); //core 1
	NxMat34 tmp1 = createTfm(0,0,-m_lidOffset,0,PI/2,0);
	createLid(lastPart->act,tmp1,true); //lid core 1
	NxMat34 tmp2 = createTfm(0,0,m_lidOffset,0,-PI/2,0);
	createLid(lastPart->act,tmp2,true); //lid core 1
	NxMat34 tmp3 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,tmp3,true); //encoder box core 1

	/*curPart=createCylinderCore(lastPart->act,createTfm(0,m_coreOffset,0,PI/2,0,getInitAngle(jointNo)),false); //core 2
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;*/
	NxMat34 tmp4 = createTfm(0,m_coreOffset,0,PI/2,0,0);
	lastPart=createCylinderCore(lastPart->act,tmp4,true); //core 2	createLid(lastPart->act,createTfm(0,0,-m_lidOffset,0,PI/2,0),true); //lid core 2
	NxMat34 tmp5 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,tmp5,true); //encoder box core 2

	//curPart=createArm(lastPart->act,createTfm(0,0,0,-PI/2,0,getInitAngle(jointNo)),false); //arm
	NxMat34 tmp6 = createTfm(0,0,0,-PI/2,0,getInitAngle(jointNo));
	curPart=createArm(lastPart->act,tmp6,false,m_armLength); //arm
	createParamMotorJoint(lastPart,NxVec3(0,1,0),curPart,jointNo++);
	lastPart=curPart;

	//curPart=createCylinderCore(lastPart->act,createTfm(0,23.5f,0,PI/2,getInitAngle(jointNo),0),false); //core 3
	NxMat34 tmp7 = createTfm(0,m_armLength*2,0,PI/2,getInitAngle(jointNo),0);
	curPart=createCylinderCore(lastPart->act,tmp7,false); //core 3
	createParamMotorJoint(curPart,NxVec3(0,-1,0),lastPart,jointNo++);
	lastPart=curPart;
	NxMat34 tmp8 = createTfm(0,0,m_lidOffset,0,-PI/2,0);
	createLid(lastPart->act,tmp8,true); //lid core 3
	NxMat34 tmp9 = createTfm(m_encBoxOffset,0,0,PI,0,0);
	createEncoderBox(lastPart->act,tmp9,true); //encoder box core 3

	//lastPart=createTip(lastPart->act,createTfm(0,0,-8,-PI/2,0,0),true); //tip
	//float tipLength=m_params->getValue("tipLength");
	NxMat34 tmp10 = createTfm(0,0,-8,-PI/2,0,0);
	lastPart=createTip(lastPart->act,tmp10,true,m_tipLength); //tip

	return lastPart;
}


X2Crawler::X2Crawler(X2CrawlerParams* params/* =NULL */,bool applyLegLength)
{
	//super constructor called automatically
	if(NULL==params)
		params=new X2CrawlerParams();
	assert(params);
	//params->print();
	m_params=params;
	//FIX: need to resolve if params should be pointer or someth else. does it copy with maps and vectors automatically?

	for(int i=0;i<3;i++)
		m_touchSensors[i]=false;
	Part* lastPart=NULL; 
	Part* curPart=NULL;
	int jointNo=0;

	m_coreOffset*=0.97f;
	//float height=40.0f;
	float height=55.0f;
	float sensorSz=1;


	//EXPERI!!
	//printf("tip length: %.2f\n",m_params->getValue("tipLength"));
	if(applyLegLength) {
		m_tipLength=m_params->getValue("tipLength");
		m_armLength=m_params->getValue("armLength");
		printf("tip len: %2.2f  arm len: %2.2f",m_tipLength,m_armLength);
	}
	else {
		m_tipLength=X2_TIPLEN;
		m_armLength=X2_ARMLEN;
	}


	NxMat34 tmp11 = createTfm(0,height+m_coreOffset/2,0,0,0,0);
	Part* coreRight=createCylinderCore(NULL,tmp11,false);
	create3Axis(coreRight,createTfm(0,0,m_coreOffset,0,PI,PI/2),jointNo);

	NxMat34 tmp12 = createTfm(-m_coreOffset,0,0,0,0,0);
	lastPart=createCylinderCore(coreRight->act,tmp12,true);
	create3Axis(lastPart,createTfm(0,0,m_coreOffset,0,0,PI/2),jointNo);

	NxMat34 tmp13 = createTfm(0,0,-m_coreOffset,0,0,0);
	Part* coreLeft=createCylinderCore(coreRight->act,tmp13,true);
	create3Axis(coreLeft,createTfm(0,0,-m_coreOffset,0,PI,PI/2),jointNo);

	NxMat34 tmp14 = createTfm(-m_coreOffset,0,0,0,0,0);
	lastPart=createCylinderCore(coreLeft->act,tmp14,true);
	create3Axis(lastPart,createTfm(0,0,-m_coreOffset,0,0,PI/2),jointNo);

	coreRight->addTouchSensor(NxVec3(0,m_coreOffset*0.8f,0),&m_touchSensors[0],sensorSz);
	coreRight->addTouchSensor(NxVec3(0,0,m_coreOffset*1.8f),&m_touchSensors[1],sensorSz);
	coreRight->addTouchSensor(NxVec3(0,0,-m_coreOffset*2.8f),&m_touchSensors[2],sensorSz);



	//increase stability
	int solverIterations=40; //4 is default
	BOOST_FOREACH(Part* part, m_robParts) //not portable
		part->act->setSolverIterationCount(solverIterations);

}




//////////////////////////////////////////////////////////////////////////
// EVOLUTION AND FITNESS
//////////////////////////////////////////////////////////////////////////

float calculateX2CrawlerFitness(GAGenome& g)
{
	X2CrawlerEvolutionHarness* evoHarness = (X2CrawlerEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;
	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	bool evolveTipLengthPhase=false;
	int runFrames=3000;
	if(evoHarness->generationNumber() >= 0) {
		evolveTipLengthPhase=true;
		runFrames=4000;
	}


	//map from genome
	//hope this works..
	X2CrawlerParams p;
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


	//X2Crawler machine(&p); //ok p not going out of scope here
	X2Crawler machine(&p,evolveTipLengthPhase); //ok p not going out of scope here


	NxVec3 firstPos=machine.getPosition();
	NxVec3 lastPos=firstPos;
	double distSum=0.0;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;

	int f;
	for(f=0;f<runFrames && !cheat;f++) {
		bool freeze=false;
		if(glfwGetKey(GLFW_KEY_F3)) freeze=true;
		if( glfwGetKey(GLFW_KEY_ESC) ) {
			evoHarness->stop();
			break; 
		}

		if(freeze && f>0) //litt dumt kanskje ha while i stedet
			f--;
		machine.update((float)simTime);
		//setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0.01f*(float)simTime);
		setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0);
		evoHarness->updateGraphicsAndPhysics(freeze);
		if(!freeze)
			simTime+=evoHarness->m_timestep;
		lastPos=machine.getPosition();

		for(int i=0;i<3;i++) {
			if(machine.m_touchSensors[i]==true) {
				cheat=true;
				break;
			}
		}


	} //end eval loop

	NxVec3 diff=machine.getPosition()-firstPos;
	terminatePhysics();
	//int framesElapsed=f;
	//float fitnessValue=(float)__max(diff.magnitude(),0); //allows any direction of final movement, not only "forward"
	float fitnessValue=fmax(diff.dot(NxVec3(1,0,0)),0); //only forward
	fitnessValue=fitnessValue*60.0f/runFrames;

	if(cheat) {
		printf(" cheat!");
		fitnessValue=0;
	}
	printf("\t\t\tfit: %f\n",fitnessValue);
	return fitnessValue;
}


//her kan det kanskje flyttes mer ut til superklasse?
X2CrawlerEvolutionHarness::X2CrawlerEvolutionHarness(int seed)
: EvolutionHarness()
{
	GARandomSeed(seed);
	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	char *paramFile="crawlersettings.txt"; //read parameters from file, overwrite default settings!
	params.read(paramFile);
	setSimulatorTimeStep(1/60.0f);
	//setSimulatorTimeStep(1/120.0f);

	//initialization of mapping, bits to decimals
	int nBits=0;
	GABin2DecPhenotype mapping;
	X2CrawlerParams p; //initializes min/max values
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

	GABin2DecGenome genome(mapping, calculateX2CrawlerFitness, (void *)this); //pass a pointer to this object
	//genome.encode(GAGrayDecode);  ? some way like that?? also specify encode
	GAPopulation pop(genome);
	pop.evaluator(forcedPopEvaluator); //this uses the custom evaluator which forces the elite to be evaluated each time (?)
	m_ga = new GASimpleGA(pop);
	if(NULL==m_ga)
		systemError("could not alloc GA");
	m_ga->parameters(params);
	stopEvolution=false;
}

X2CrawlerEvolutionHarness::~X2CrawlerEvolutionHarness()
{
	if(gPhysicsSDK)
		terminatePhysics();
}



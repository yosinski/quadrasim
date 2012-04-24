#include "legmachine.h"
#include "../physx/MyCloth.h"

//for evolution:
#include "../evolutionharness.h"
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#include "../physics.h"
#include "../graphics/graphics.h"
#include "../base/system.h"
#include "muscle.h"
#define cout STD_COUT

Part* LegMachine::getCentralPart()
{
	return NULL;
}

float LegMachine::createSingleLeg(Part* platform,Part** legParts,NxVec3 localAttachPoint,LegParams& params)
{
	NxActor* attachTo=platform->act;
	float platY=attachTo->getGlobalPose().t.y;
	NxVec3 platDims=((NxBoxShape*)attachTo->getShapes()[0])->getDimensions();

	float topLen=params.getValue("topLegLen");
	float botLen=params.getValue("botLegLen");
	//float m_legWidth=0.6f;
	float jointSpacingTop=0.7f;
	float jointSpacingBot=1.2f;
	float topRot=params.getValue("topLegRot");
	float botRot=params.getValue("botLegRot");
	//float botOffset=0.25f; //how much lower leg is "pulled back/forward" (0 - 0.5?)
	float botOffset=params.getValue("botLegOffset");
	float botOffsetLen=botOffset*botLen+m_legWidth;

	NxVec3 globalAttachPoint;
	float topLegTop=platY-platDims.y-jointSpacingTop;
	attachTo->getGlobalPose().multiply(localAttachPoint,globalAttachPoint);
	Part* topLeg=createCapsulePart(Point3D(0,topLegTop,globalAttachPoint.z),topLen*2,true,m_legWidth);
	NxActor* topAct=topLeg->act;
	topAct->getShapes()[0]->setLocalPosition(NxVec3(0,-topLen-m_legWidth,0));  //offset center
	NxMat33 rot;
	rot.rotZ(topRot);
	topAct->setGlobalOrientation(rot); //rotate around new center

	//bottom leg part
	//find center point of bottom leg by transforming with top leg pose
	NxVec3 botTrans(0,-topLen*2-m_legWidth*2-jointSpacingBot-m_legWidth,0); //translation relative to top leg top
	NxVec3 globalBotLegCenter;
	topAct->getGlobalPose().multiply(botTrans,globalBotLegCenter);
	Part* botLeg=createCapsulePart(globalBotLegCenter,botLen*2,true,m_legWidth);
	NxActor* botAct=botLeg->act;
	botAct->getShapes()[0]->setLocalPosition(NxVec3(0,-botOffsetLen,0));  //offset center
	rot.rotZ(botRot);
	botAct->setGlobalOrientation(rot); //rotate around new center

	//calculate lowest point on leg..
	NxVec3 localTipBotLeg(0,-botLen-botOffsetLen,0);
	NxVec3 globalTipBotLeg;
	botAct->getGlobalPose().multiply(localTipBotLeg,globalTipBotLeg);
	globalTipBotLeg.y-=m_legWidth;


	float limLow=-0.35f*PI; //should be negative 
	float limHigh=0.15f*PI; //should be positive (joint arrow is purple)
	//now using top leg top as top joint center - should be ok
	createRevoluteJoint(topPart->act,topLeg->act,topAct->getGlobalPose().t,NxVec3(0,0,1),limLow,limHigh);

	limLow=-0.35f*PI;
	limHigh=0.2f*PI;
	//now using bot leg center as bot joint center - should be ok
	createRevoluteJoint(topLeg->act,botLeg->act,globalBotLegCenter,NxVec3(0,0,1),limLow,limHigh);

	legParts[0]=topLeg;
	legParts[1]=botLeg;

	return globalTipBotLeg.y; //return offset above ground, for height adjustments

}

void LegMachine::attachMuscles(Part* platform,Part** legPointers,Muscle** musclePointers,NxVec3 localAttachPoint,LegParams& params,int muscleType)
{
	NxVec3 platDims=((NxBoxShape*)platform->act->getShapes()[0])->getDimensions();
	float topLen=params.getValue("topLegLen");
	float botLen=params.getValue("botLegLen");
	Part* topLeg=legPointers[0];
	Part* botLeg=legPointers[1];
	NxActor* attachTo=platform->act;
	float botOffset=params.getValue("botLegOffset");
	float botOffsetLen=botOffset*botLen+m_legWidth;

	//float musAttTopLegBack=params.getValue("muscleAttachPointTopLegBack");
	//float musAttTopLegFront=params.getValue("muscleAttachPointTopLegFront");
	float topMusAttTopLegBack=params.getValue("topMuscleAttachPointTopLegBack");
	float topMusAttTopLegFront=params.getValue("topMuscleAttachPointTopLegFront");
	float botMusAttTopLegBack=params.getValue("botMuscleAttachPointTopLegBack");
	float botMusAttTopLegFront=params.getValue("botMuscleAttachPointTopLegFront");
	//botLeg back alltid på tuppen
	float musAttBotLegFront=params.getValue("muscleAttachPointBotLegFront");
	NxVec3 localAttachBackPlatform(-platDims.x,0,localAttachPoint.z);
	NxVec3 localAttachFrontPlatform(platDims.x,0,localAttachPoint.z);
	NxVec3 localUpperAttachBackTopLeg(-m_legWidth,-m_legWidth-topLen+topMusAttTopLegBack*topLen,0); //relative to the "center" of the leg
	NxVec3 localUpperAttachFrontTopLeg(m_legWidth,-m_legWidth-topLen+topMusAttTopLegFront*topLen,0);
	NxVec3 localLowerAttachBackTopLeg(-m_legWidth,-m_legWidth-topLen-botMusAttTopLegBack*topLen,0);
	NxVec3 localLowerAttachFrontTopLeg(m_legWidth,-m_legWidth-topLen-botMusAttTopLegFront*topLen,0);
	NxVec3 localAttachFrontBotLeg(m_legWidth,-musAttBotLegFront*botLen-botOffsetLen,0);
	NxVec3 localAttachBackBotLeg(m_legWidth,botLen-botOffsetLen,0); //at the back tip

	for(int i=0;i<4;i++)
		musclePointers[i]=NULL;

	float stretchAmount=0.045f;
	//float stretchAmount=0.03f;
	if(DISTANCE_MUSCLE==muscleType) {
		DistanceJointMuscle *muscles[4]={0};
		muscles[0]=new DistanceJointMuscle(topLeg->act,localUpperAttachFrontTopLeg,attachTo,localAttachFrontPlatform,stretchAmount);
		muscles[2]=new DistanceJointMuscle(botLeg->act,localAttachFrontBotLeg,topLeg->act,localLowerAttachFrontTopLeg,stretchAmount); 
		for(int i=0;i<4;i++)
			musclePointers[i]=muscles[i];
	}
	else if(SPRING_MUSCLE==muscleType) {
		SpringMuscle* muscles[4]={0};
		//float spring=8000.0f;
		float spring=5000.0f;
		//float damper=0.7f;
		float damper=0.5f;
		//needs some adjustments in positions here
		muscles[0]=new SpringMuscle(topLeg->act,localUpperAttachFrontTopLeg,attachTo,localAttachFrontPlatform,stretchAmount,spring,damper);
		muscles[1]=new SpringMuscle(topLeg->act,localUpperAttachBackTopLeg,attachTo,localAttachBackPlatform,stretchAmount,spring,damper);
		muscles[1]->setStretchLength(muscles[0]->getStretchLength());
		muscles[2]=new SpringMuscle(botLeg->act,localAttachFrontBotLeg,topLeg->act,localLowerAttachFrontTopLeg,stretchAmount,spring,damper); 
		muscles[3]=new SpringMuscle(botLeg->act,localAttachBackBotLeg,topLeg->act,localLowerAttachBackTopLeg,stretchAmount,spring,damper); 
		muscles[3]->setStretchLength(muscles[2]->getStretchLength());
		for(int i=0;i<4;i++)
			musclePointers[i]=muscles[i];
	}
	else if(CLOTH_MUSCLE==muscleType) {
		float musWidth=1.4f;
		//float musWidth=1.0f;
		//int heightDivs=6;
		//int aroundDivs=40;
		int heightDivs=6;
		int aroundDivs=32;
		ClothMuscle *muscles[4]={0};
		muscles[0]=new ClothMuscle(topLeg->act,localUpperAttachFrontTopLeg,attachTo,localAttachFrontPlatform,musWidth,heightDivs,aroundDivs);
		muscles[1]=new ClothMuscle(topLeg->act,localUpperAttachBackTopLeg,attachTo,localAttachBackPlatform,musWidth,heightDivs,aroundDivs); //z value needs to be found
		muscles[2]=new ClothMuscle(botLeg->act,localAttachFrontBotLeg,topLeg->act,localLowerAttachFrontTopLeg,musWidth,heightDivs,aroundDivs); 
		muscles[3]=new ClothMuscle(botLeg->act,localAttachBackBotLeg,topLeg->act,localLowerAttachBackTopLeg,musWidth,heightDivs,aroundDivs); 
		for(int i=0;i<4;i++)
			musclePointers[i]=muscles[i];
	}

}

void LegMachine::attachTouchSensor(Part* leg, bool* touchSensor, LegParams& params)
{
	float botOffset=params.getValue("botLegOffset");
	float botLen=params.getValue("botLegLen");
	float botOffsetLen=botOffset*botLen+m_legWidth;
	//NxVec3 localAttachFrontBotLeg(0,-botLen-botOffsetLen-m_legWidth,0);
	NxVec3 localAttachFrontBotLeg(m_legWidth/2,-botLen-botOffsetLen-m_legWidth,0);
	//float sensorSz=0.5f;
	//float sensorSz=m_legWidth*0.8f;
	float sensorSz=m_legWidth*0.5f;
	leg->addTouchSensor(localAttachFrontBotLeg,touchSensor,sensorSz);
}

LegMachine::LegMachine(LegParams& params,int muscleType)
{
	for(int i=0;i<8;i++)
		muscles[i]=NULL;
	for(int i=0;i<4;i++)
		legParts[i]=NULL;
	for(int i=0;i<2;i++)
		touchSensors[i]=true; //if not on ground in beginning don't record it

	m_legWidth=0.6f;

	float centerDistance=100.0f;
	//float platLen=5.0f;
	float platLen=params.getValue("platLen");
	float platThickness=1.0f;
	//float platHeight=18.0f;
	float platHeight=27.0f;
	float depth=4.5f;
	float machineZ=centerDistance+depth;

	//platform
	topPart=createBoxPart(Point3D(0,platHeight,machineZ),Point3D(platLen,platThickness,depth));

	//float sensorSz=0.5f;
	//topPart->addTouchSensor(NxVec3(-platLen,-platThickness,-depth),&touchSensors[0],sensorSz);
	//topPart->addTouchSensor(NxVec3(platLen,-platThickness,-depth),&touchSensors[1],sensorSz);
	//topPart->addTouchSensor(NxVec3(-platLen,-platThickness,depth),&touchSensors[2],sensorSz);
	//topPart->addTouchSensor(NxVec3(platLen,-platThickness,depth),&touchSensors[3],sensorSz);

	//2 legs
	float legHeightOffs;
	createSingleLeg(topPart,&legParts[0],NxVec3(0,0,depth-1.0f),params);
	legHeightOffs=createSingleLeg(topPart,&legParts[2],NxVec3(0,0,-depth+1.0f),params);

	//hack down the position of the parts to make it stand on ground
	for(unsigned i=0;i<parts.size();i++) {
		parts[i]->act->setGlobalPosition(parts[i]->act->getGlobalPosition()+NxVec3(0,-legHeightOffs,0));
	}
	platHeight-=legHeightOffs;
	

	//cloth cannot be moved (easily?) so they have to be created after the height adjustment
	attachMuscles(topPart,&legParts[0],&muscles[0],NxVec3(0,0,depth-1.0f),params,muscleType);
	attachMuscles(topPart,&legParts[2],&muscles[4],NxVec3(0,0,-depth+1.0f),params,muscleType);

	attachTouchSensor(legParts[1],&touchSensors[0],params);
	attachTouchSensor(legParts[3],&touchSensors[1],params);

	//walking harness - rods linking to center
	float secondRadRodOffset=4.0f;
	float rodWidth=0.6f;
	float centerRodWidth=6*rodWidth;
	float centerRodHeight=platHeight;

	//float centerJointLimit=0.001f*PI; //limited movement  (depends on distance to center)
	float centerJointLimit=0.1f*PI; //free movement  (depends on distance to center)
	Part* centerRod=createCapsulePart(Point3D(0,0,0),centerRodHeight,false,centerRodWidth);
	//link center rod to world
	NxRevoluteJoint* centerJoint=createRevoluteJoint(centerRod->act,NULL,NxVec3(0,centerRodHeight/2,0),NxVec3(0,1,0));

	//first radial rod
	float radRodLength=centerDistance-4*centerRodWidth;
	Part* radRod=createCapsulePart(Point3D(0,platHeight,centerDistance/2),radRodLength,true,rodWidth);
	NxMat33 rot;
	rot.rotX(PI/2);
	radRod->act->setGlobalOrientation(rot);
	//link radial rod to center rod
	createRevoluteJoint(radRod->act,centerRod->act,NxVec3(0,platHeight,0),NxVec3(1,0,0),-centerJointLimit,centerJointLimit);
	//link machine to radial rod
	createRevoluteJoint(topPart->act,radRod->act,NxVec3(0,platHeight,centerDistance),NxVec3(1,0,0),-centerJointLimit,centerJointLimit);

	//second rod
	//Part* radRod2=createCapsulePart(Point3D(0,platHeight+secondRadRodOffset,0),radRodLength*2,true,rodWidth);
	Part* radRod2=createCapsulePart(Point3D(0,platHeight+secondRadRodOffset,centerDistance/2),radRodLength,true,rodWidth);
	radRod2->act->setGlobalOrientation(rot);
	createRevoluteJoint(radRod2->act,centerRod->act,NxVec3(0,platHeight+secondRadRodOffset,0),NxVec3(1,0,0),-centerJointLimit,centerJointLimit);
	createRevoluteJoint(topPart->act,radRod2->act,NxVec3(0,platHeight+secondRadRodOffset,centerDistance),NxVec3(1,0,0),-centerJointLimit,centerJointLimit);

	//need to increase the stability of the joints!!
	int solverIterations=50;
	//quick hack:
	for(unsigned i=0;i<parts.size();i++) {
		parts[i]->act->setSolverIterationCount(solverIterations);
	}

	//another option to damping could be setting the intertia tensor(?)
	//extreme damping to make it more difficult to rotate
	float damping=25.0f; //seems stable 
	//float damping=100.0f;
	centerRod->act->setAngularDamping(damping);
	radRod->act->setAngularDamping(damping);
	radRod2->act->setAngularDamping(damping);



	//take a local copy of the params to use for control function
	m_params=params; //the map seems to copy itself automagically 
}



void LegMachine::update(float simulationTime)
{
	float phaseOtherLeg=m_params.getValue("phaseOtherLeg");
	float phaseBelowKnee=m_params.getValue("phaseBelowKnee");

	float frequency=m_params.getValue("freq");
	float t=frequency*simulationTime;

	static FILE* controllerOut=NULL;
	if(controllerOut==NULL) {
		controllerOut=fopen("sin_controller_out.txt","w");
	}

	//upper part leg 1
	float t0=0;
	float t1=m_params.getValue("attackTop");
	float t2=t1+m_params.getValue("pauseTop");
	float t3=__min(t2+m_params.getValue("decayTop"),0.99f);
	

	t+=t1/2.0f; //want to start with muscles in neutral position

	float remainder=fmod(t,1.0f);
	float pressure=sineEnvelope(remainder,t0,t1,t2,t3);
	if(muscles[0]) muscles[0]->setPull(pressure);
	if(muscles[1]) muscles[1]->setPull(1.0f-pressure);
	fprintf(controllerOut,"%.3f %.3f ",simulationTime,pressure);

	//upper part leg 2
	remainder=fmod(t+phaseOtherLeg,1.0f);
	pressure=sineEnvelope(remainder,t0,t1,t2,t3);
	if(muscles[4]) muscles[4]->setPull(pressure);
	if(muscles[5]) muscles[5]->setPull(1.0f-pressure);

	//lower part leg 1
	t0=0;
	t1=m_params.getValue("attackBot");
	t2=t1+m_params.getValue("pauseBot");
	t3=__min(t2+m_params.getValue("decayBot"),0.99f);
	remainder=fmod(t+phaseBelowKnee,1.0f);
	pressure=sineEnvelope(remainder,t0,t1,t2,t3);
	if(muscles[2]) muscles[2]->setPull(pressure);
	if(muscles[3]) muscles[3]->setPull(1.0f-pressure);
	fprintf(controllerOut,"%.3f ",pressure);

	//lower part leg 2
	remainder=fmod(t+phaseBelowKnee+phaseOtherLeg,1.0f);
	pressure=sineEnvelope(remainder,t0,t1,t2,t3);
	if(muscles[6]) muscles[6]->setPull(pressure);
	if(muscles[7]) muscles[7]->setPull(1.0f-pressure);

	//can read positions here?
	//remember to project position relative to forward vector...
	NxVec3 topPos=topPart->act->getGlobalPose().t;
	NxVec3 sensorPos=legParts[1]->act->getShapes()[1]->getGlobalPose().t;

	NxVec3 up(0,1,0);
	NxVec3 forward=up.cross(topPos);
	forward.normalize();
	NxVec3 delta=sensorPos-topPos;
	float forwardAmount=delta.dot(forward);
	fprintf(controllerOut,"%.3f %.3f ",forwardAmount,sensorPos.y);


	fprintf(controllerOut,"\n");

}




//////////////////////////////////////////////////////////////////////////////////
///evoLeg stuff

float calculateLegFitness(GAGenome& g)
{
	LegEvolutionHarness* evoHarness = (LegEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;
	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	//set up Leg from genome
	LegParams p;
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

	//LegMachine machine(p,LegMachine::DISTANCE_MUSCLE);
	LegMachine machine(p,LegMachine::SPRING_MUSCLE);
	//LegMachine machine(p,LegMachine::CLOTH_MUSCLE);


	NxVec3 firstPos=machine.topPart->act->getGlobalPose().t;
	NxVec3 lastPos=firstPos;
	double distSum=0.0;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;
	const int runFrames=700;
	float yVals[runFrames]={0};
	int touchCount[2]={0};
	bool touchVals[runFrames][2]={0};

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
		setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0);
		evoHarness->updateGraphicsAndPhysics(freeze);
		if(!freeze)
			simTime+=evoHarness->m_timestep;

		//check how much advance in the forward direction
		NxVec3 curPos=machine.topPart->act->getGlobalPose().t;
		NxVec3 up(0,1,0);
		NxVec3 forward=up.cross(curPos);
		forward.normalize();
		NxVec3 delta=curPos-lastPos;
		float deltaLength=delta.dot(forward);
		distSum+=deltaLength;
		lastPos=curPos;


		//try to detect jumping/floating
		//FIX: right constants?
		float y=machine.topPart->act->getGlobalPose().t.y;
		if(y > firstPos.y*1.2f || y < firstPos.y*0.75f) {
			printf("too high/low y pos ");
			cheat=true;
			break;
		}
			
		yVals[f]=y;		

		for(int i=0;i<2;i++) {
			if(machine.touchSensors[i])
				touchCount[i]++;
			
			touchVals[f][i]=machine.touchSensors[i];
		}

		//check if dragging for last N frames
		//now only for one leg
		const unsigned dragMeasurePeriod=200; 
		if(f>dragMeasurePeriod) {
			int nTouch=0;
			for(int i=f-dragMeasurePeriod;i<f;i++) {
				if(touchVals[i][0])
					nTouch++;
			}
			float touchFraction=float(nTouch)/dragMeasurePeriod;
			if(touchFraction>0.98f) {
				printf("continuous drag check: %f -> dragging",touchFraction);
				cheat=true;
				break;
			}
		}

		//kan også gjøre denne sjekke for last N frames, som over. Kanskje bedre
		//check if too high speed (can only do after a certain time)
		if(f>150) {
			//float speed=fabs((float)distSum)/f;
			float speed=float(distSum/f);
			if(speed>0.07f) {
			//if(speed>0.12f) {
				printf("too high speed: %f, probably exploiting unknown simulator feature ",speed);
				cheat=true;
				break;
			}
			if(speed<0.005f) {
				printf("too low speed: %f, can cut evaluation ",speed);
				cheat=true;
				break;
			}
		}
			
	} //end eval loop
	//NxVec3 diff=machine.topPart->physicsEquivalent->getGlobalPose().t-firstPos;
	terminatePhysics();
	int framesElapsed=f;
	float fitnessValue=(float)__max(distSum,0);
	//float fitnessValue=fabs((float)distSum);


	for(int i=0;i<2;i++) {
		float amount=(float)touchCount[i]/runFrames;
		printf("%.2f ",amount);
		if(amount > 0.95f) {
			cheat=true;
			printf(" drag/no move ");
		}
		/*if(amount < 0.05f ) {
			cheat=true;
			printf(" floating ");
		}*/
	}
	printf("\n");


	float avgY=0;
	for(int f=0;f<framesElapsed;f++) {
		avgY+=yVals[f];
	}
	avgY/=framesElapsed;

	float stdDev=0;
	for(int f=0;f<framesElapsed;f++) {
		float d=yVals[f]-avgY;
		stdDev+=d*d;
	}
	stdDev/=framesElapsed;
	stdDev=sqrt(stdDev);
	printf("y std dev: %.3f",stdDev);

	//avoid jumping:
	if(stdDev>0.4f) {
		printf(" too high std dev, maybe jumping? ");
		//cheat=true;
		fitnessValue*=0.5f; //penalty but not killing completely
	}

	//new version:
	/*fitnessValue-=(stdDev*stdDev)*10.0f;
	fitnessValue=(float)__max(0,fitnessValue);*/



	if(cheat) {
		printf("cheat!");
		fitnessValue=0;
	}

	printf("\nfit: %f\n",fitnessValue);
	return fitnessValue;
}



//her kan det kanskje flyttes mer ut til superklasse?
//sjekk hvordan det kan fikses
LegEvolutionHarness::LegEvolutionHarness(int seed)
: EvolutionHarness()
{
	GARandomSeed(seed);
	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	char *paramFile="settings.txt"; //read parameters from file, overwrite default settings!
	params.read(paramFile); 

	//mapping of bits to decimals
	GABin2DecPhenotype mapping;
	LegParams p; //initializes min/max values
	//må ha iterator..
	std::map<std::string, FloatParam>::iterator it;
	for(it=p.paramList.begin(); it!=p.paramList.end(); it++) {
		mapping.add(it->second.precision, it->second.minVal, it->second.maxVal); //specified precision
	}

	GABin2DecGenome genome(mapping, calculateLegFitness, (void *)this); //pass a pointer to this object
	GAPopulation pop(genome);
	pop.evaluator(forcedPopEvaluator); //this uses the custom evaluator which forces the elite to be evaluated each time (?)
	m_ga = new GASimpleGA(pop);
	if(NULL==m_ga)
		systemError("could not alloc GA");
	m_ga->parameters(params);
	stopEvolution=false;
}

LegEvolutionHarness::~LegEvolutionHarness()
{
	if(gPhysicsSDK)
		terminatePhysics();
}

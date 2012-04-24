#include "quadromachine.h"

#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#define cout STD_COUT

#include <assert.h>
#include <stdlib.h>
#include <sstream>
#include <istream>
#include "../physics.h"
#include "../graphics/MeshGraphicsObject.h"
#include "../base/system.h"
#include "../graphics/graphics.h"

#define PRINT_STATS 1 //for printing controller and position stats
#define PRINT_INTERVAL 0.1 //seconds between each log line

int minMaxAngles[][2] = {
	{150,770},
	{30,940},
	{150,770},
	{30,940},
	{150,770},
	{30,940},
	{150,770},
	{30,940},
	{512-180,512+180},
};


//these are class static to speed up loading time
MeshGraphicsObject* QuadroMachine::m_pcPartMesh=NULL;
MeshGraphicsObject* QuadroMachine::m_batteryPartMesh=NULL;
MeshGraphicsObject* QuadroMachine::m_upperLegMesh=NULL;
MeshGraphicsObject* QuadroMachine::m_lowerLegMesh=NULL;

#define SC 1.0f //scale of robot dimensions - originally in cm, sc==0.01 -> physics units are [m]
//nå går den litt gjennom bakken hvis den er for liten - er nødt til å stille inn noe i grafikk (depth buffer)?

//sjekk gjennom kode med p-kontroll
//vurdere 6DOF
//rydde opp mer i testobjects

//shape constants
float hingeXOffs=-5.2f*SC;
float innerYOffs=3.0f*SC;
float outerYOffs=6.0f*SC;
float innerZOffs=6.2f*SC;
float outerZOffs=3.0f*SC;

//gjøre om til meter
//printe ut hvor langt den har gått (og hastighet)

Part* QuadroMachine::createPCCore(NxActor* attachTo, NxMat34& transform, bool glue)
{
	//note! coordinate system maybe not as you think!
	NxActorDesc actorDesc;
	actorDesc.density=1.0f; //this may have to be adjusted
	actorDesc.userData=m_pcPartMesh;

	//simplified frame + pc
	NxBoxShapeDesc vertBox;
	vertBox.dimensions.set(  SC*5.4f,SC*5.8f,SC*1.8f);
	vertBox.localPose.t.set(SC*-3.8f,SC*0.0f,SC*1.8f);
	actorDesc.shapes.push_back(&vertBox);

	//shape for hinges
	NxBoxShapeDesc hingeBox;
	hingeBox.dimensions.set(  SC*1.4f,SC*0.2f,SC*3.5f);

	NxBoxShapeDesc b1=hingeBox;
	b1.localPose.t.set(hingeXOffs,-outerYOffs,outerZOffs);
	b1.localPose.M.rotX(PI/4);
	actorDesc.shapes.push_back(&b1);

	NxBoxShapeDesc b2=b1;
	b2.localPose.t.set(hingeXOffs,-innerYOffs,innerZOffs);
	actorDesc.shapes.push_back(&b2);

	NxBoxShapeDesc b3=hingeBox;
	b3.localPose.t.set(hingeXOffs,outerYOffs,outerZOffs);
	b3.localPose.M.rotX(-PI/4);
	actorDesc.shapes.push_back(&b3);

	NxBoxShapeDesc b4=b3;
	b4.localPose.t.set(hingeXOffs,innerYOffs,innerZOffs);
	actorDesc.shapes.push_back(&b4);
	
	/*//post-scale down all the boxes, to have units in [m]
	for(unsigned i=0;i < actorDesc.shapes.size();i++) {
		assert(actorDesc.shapes[i]->getType()==NX_SHAPE_BOX); //this will only work if all shapes are boxes!
		NxBoxShapeDesc* bd=(NxBoxShapeDesc*)actorDesc.shapes[i];
		bd->dimensions=bd->dimensions*0.1f;
	}*/

	return createMachinePart(actorDesc,attachTo,transform,glue);
}



Part* QuadroMachine::createBatteryCore(NxActor* attachTo, NxMat34& transform, bool glue)
{
	NxActorDesc actorDesc;
	actorDesc.density=1.0f; //this may have to be adjusted
	actorDesc.userData=m_batteryPartMesh;
	//simplified frame + battery
	NxBoxShapeDesc vertBox;
	vertBox.dimensions.set(  SC*5.4f,SC*5.8f,SC*1.8f);
	vertBox.localPose.t.set(SC*-3.8f,SC*0.0f,SC*1.8f);
	actorDesc.shapes.push_back(&vertBox);

	//shape for hinges
	NxBoxShapeDesc hingeBox;
	hingeBox.dimensions.set(  SC*1.4f,SC*0.2f,SC*3.5f);

	NxBoxShapeDesc b1=hingeBox;
	b1.localPose.t.set(hingeXOffs,-outerYOffs,outerZOffs);
	b1.localPose.M.rotX(PI/4);
	actorDesc.shapes.push_back(&b1);

	NxBoxShapeDesc b2=b1;
	b2.localPose.t.set(hingeXOffs,-innerYOffs,innerZOffs);
	actorDesc.shapes.push_back(&b2);

	NxBoxShapeDesc b3=hingeBox;
	b3.localPose.t.set(hingeXOffs,outerYOffs,outerZOffs);
	b3.localPose.M.rotX(-PI/4);
	actorDesc.shapes.push_back(&b3);

	NxBoxShapeDesc b4=b3;
	b4.localPose.t.set(hingeXOffs,innerYOffs,innerZOffs);
	actorDesc.shapes.push_back(&b4);

	return createMachinePart(actorDesc,attachTo,transform,glue);
}


Part* QuadroMachine::createUpperLeg(NxActor* attachTo, NxMat34& transform, bool glue)
{
	NxActorDesc actorDesc;
	actorDesc.density=1.0f; //this may have to be adjusted
	actorDesc.userData=m_upperLegMesh;

	NxBoxShapeDesc vertBox;
	vertBox.dimensions.set(  SC*1.2f,SC*1.8f,SC*1.4f);
	vertBox.localPose.t.set(0,SC*1.4f,0);
	actorDesc.shapes.push_back(&vertBox);

	//"outer" geometry
	NxBoxShapeDesc hingeBox;
	hingeBox.dimensions.set(  SC*0.8f,SC*1.6f,SC*0.2f);
	hingeBox.localPose.M.rotZ(-PI/4);

	NxBoxShapeDesc b1=hingeBox;
	b1.localPose.t.set(SC*-0.8f,SC*9,SC*-2.5f);
	actorDesc.shapes.push_back(&b1);

	NxBoxShapeDesc b2=hingeBox;
	b2.localPose.t.set(SC*-0.8f,SC*9,SC*2.5f);
	actorDesc.shapes.push_back(&b2);

	//"inner" geometry - not very accurate now
	hingeBox.dimensions.set(  SC*1.1f,SC*1.6f,SC*0.2f);
	hingeBox.localPose.M.rotZ(PI/8);

	NxBoxShapeDesc b3=hingeBox;
	b3.localPose.t.set(SC*-0.8f,SC*6,SC*-2.5f);
	actorDesc.shapes.push_back(&b3);

	NxBoxShapeDesc b4=hingeBox;
	b4.localPose.t.set(SC*-0.8f,SC*6,SC*2.5f);
	actorDesc.shapes.push_back(&b4);

	return createMachinePart(actorDesc,attachTo,transform,glue);
}

Part* QuadroMachine::createLowerLeg(NxActor* attachTo, NxMat34& transform, bool glue)
{
	NxActorDesc actorDesc;
	actorDesc.density=1.0f; //this may have to be adjusted
	actorDesc.userData=m_lowerLegMesh;

	NxBoxShapeDesc vertBox;
	vertBox.dimensions.set(  SC*1.2f,SC*1.8f,SC*1.4f);
	vertBox.localPose.t.set(0,SC*1.4f,0);
	actorDesc.shapes.push_back(&vertBox);

	NxBoxShapeDesc b1;
	b1.dimensions.set(  SC*1.6f,SC*0.8f,SC*0.9f);
	b1.localPose.t.set(0,SC*4.0f,0);
	actorDesc.shapes.push_back(&b1);

	NxBoxShapeDesc b2;
	b2.dimensions.set(  SC*0.6f,SC*3.0f,SC*0.9f);
	b2.localPose.t.set(SC*-0.3f,SC*7.5f,0);
	b2.localPose.M.rotZ(-PI/16);
	actorDesc.shapes.push_back(&b2);

	NxBoxShapeDesc b3;
	b3.dimensions.set(  SC*0.35f,SC*1.6f,SC*0.9f);
	b3.localPose.t.set(SC*1,SC*12.0f,0);
	b3.localPose.M.rotZ(-PI/8);
	actorDesc.shapes.push_back(&b3);

	return createMachinePart(actorDesc,attachTo,transform,glue);
}

Part* QuadroMachine::createLeg(NxActor* attachTo, NxMat34& transform, bool glue,int jointNumber)
{
	Part* lastPart=NULL;
	Part* curPart=NULL;

	//upper leg part and joint to body
	curPart=createUpperLeg(attachTo,transform,glue);
	if(attachTo)
		createParamMotorJoint(curPart->act,NxVec3(0,0,1),attachTo,jointNumber);
	lastPart=curPart;

	//lower leg and joint to upper leg
	curPart=createLowerLeg(lastPart->act,createTfm(SC*-0.4f,SC*10,SC*0,0,0,getInitAngle(jointNumber+1)),false);
	createParamMotorJoint(curPart->act,NxVec3(0,0,1),lastPart->act,jointNumber+1);

	return lastPart;
}


//no params for now
QuadroMachine::QuadroMachine(QuadroParams* params/* =NULL */)
{
	if(NULL==params)
		params=new QuadroParams();
	assert(params);
	//params->print();
	m_params=params;
	playbackMode=false;

	initMeshes();

	Part* lastPart=NULL;
	Part* curPart=NULL;

	//one side with legs
	Part* pcPart=lastPart=createPCCore(NULL,createTfm(0,SC*40,0,0,0,PI/2),false);
	createLeg(lastPart->act,createTfm(SC*-5.5f,SC*7,SC*6.4f,PI/4,PI,getInitAngle(0)),false,0);
	createLeg(lastPart->act,createTfm(SC*-5.5f,SC*-7,SC*6.4f,3*PI/4,PI,getInitAngle(2)),false,2);

	//other side with legs
	Part* batteryPart=curPart=createBatteryCore(lastPart->act,createTfm(0,0,0,PI,0,getInitAngle(8)),false); 
	lastPart=curPart;
	createLeg(lastPart->act,createTfm(SC*-5.5f,SC*7,SC*6.4f,PI/4,PI,getInitAngle(4)),false,4);
	createLeg(lastPart->act,createTfm(SC*-5.5f,SC*-7,SC*6.4f,3*PI/4,PI,getInitAngle(6)),false,6);

	//add joint between pc and battery
	createParamMotorJoint(batteryPart->act,NxVec3(0,0,1),pcPart->act,8);

	//can add sensors here
	//can increase solver accuracy here
	int solverIterations=10; //4 is default
	for each(Part* part in m_robParts) //syntax not portable
		part->act->setSolverIterationCount(solverIterations);

}


//change limLow/high to initAngle + amp? easier to specify after all?
//NB! changed params from Part to NxActor
//need to revise this function
NxRevoluteJoint* QuadroMachine::createParamMotorJoint(NxActor* p1,NxVec3 p1LocalAxis,NxActor* p2,int jointParamNumber)
{
	float maxForce=10000000;
	//float maxForce=10000;
	NxRevoluteJoint* joint;
	NxVec3 globalAxis;
	float paramLow=m_params->getValue("limLow",jointParamNumber);
	float paramHigh=m_params->getValue("limHigh",jointParamNumber);
	float limAmp=(paramHigh+paramLow)/2*PI;

	p1->getGlobalOrientation().multiply(p1LocalAxis,globalAxis);
	joint=createRevoluteMotorJoint(p1,p2,p1->getGlobalPose().t,globalAxis,-limAmp,limAmp,maxForce);
	m_robJoints.push_back(joint);
	return joint;
}


float QuadroMachine::getInitAngle(int jointNumber)
{
	float paramLow=m_params->getValue("limLow",jointNumber);
	float paramHigh=m_params->getValue("limHigh",jointNumber);
	float initAngle=(-paramLow+paramHigh)/2*PI;
	return initAngle;
}


void QuadroMachine::update(float simulationTime)
{
	if(playbackMode)
		playback(simulationTime);
	else
		control(simulationTime); //normal case
}

void QuadroMachine::loadPlaybackFile(const char* fileName)
{
	std::ifstream inFile(fileName,std::ios::in);
	if(!inFile.good()) 
		systemError("could not open file");
	printf("loading playback from %s\n",fileName);
	
	char line[400];
	int numLines=0;
	while(!inFile.eof()) {
		if(inFile.peek()=='#') { //comment line
			inFile.getline(line,400);
		}
		else {
			PositionKey p;
			inFile >> p.t;
			for(int i=0;i<NUM_JOINTS;i++) 
				inFile >> p.values[i];
			inFile.getline(line,400); //skip the rest of the line

			printf("scanned: t:%f : ",p.t);
			for(int i=0;i<NUM_JOINTS;i++) {
				printf("%.2f ",p.values[i]);
			}
			printf("\n");
			m_positions.push_back(p);
			
		}
		numLines++;
	}
	printf("read %d position keys\n",m_positions.size());
	printf("machine in playback mode\n");
	playbackMode=true;
}


float QuadroMachine::servoToAngle(float servoPos)
{
	return float(servoPos/1023.0*2*PI-PI);
}

float QuadroMachine::angleToServo(float angle)
{
	return float((angle+PI)/(2*PI)*1023.0);
}

void QuadroMachine::playback(float simulationTime)
{
	//todo:
	//FIX ANGLE CONVERSIONS! 1024 != +PI! servoen går ikke hele veien rundt. er det settings på dette?
	//- compare plots
	//- simplify function
	//- go through robot joint number definitions (compare to manual) and check with simulator. 
	//- check why some joint arcs show up "wrong" way
	//- joints seems to clamp now, probably because of already set joint limits differ from what's played back
	//  - the problem is that the motor joint (initial angle and limits) is specified according to the quadroparams
	//  - the real robot always has 0 at straight legs, while the current sim robot has initial angle derived from min/max angles
	//  - either change simulation scheme to always have "neutral" robot with initial angle always same, or just change values in default params 
	//  - if staying with relative angles on robot, hard to read out results. better to have control-internal values which are converted to robot abs values
	// fra jasons webside: Each motor may be commanded to an integer position from 0 to 1023, corresponding to an angular movement from -120° to 120°.
	// ax12-dok sier 300 grader operating angle

	//experimental:
	float timeScale=0.25f;
	simulationTime*=timeScale;
	//we could instead speed up the simulated motor, check which parameters are ok for this. p const, max speed or max force?

	if(m_positions.size()==0) {
		printf("no stored position keys\n");
		return;
	}

	static float nextIntervalPrint=0;
	bool debugPrint=false;
	if(PRINT_STATS) {
		if(simulationTime>nextIntervalPrint) {
			debugPrint=true;
			nextIntervalPrint+=PRINT_INTERVAL;
		}
	}
	
	static FILE* controllerOut=NULL;
	if(debugPrint) {
		if(controllerOut==NULL) {
			controllerOut=fopen("sin_controller_out.txt","w");
			fprintf(controllerOut,"#simtime (logvalue limitedcontrolvalue actualjointangle)x%d\n",m_robJoints.size());
			printf("logging positions\n");
		}
		fprintf(controllerOut,"%.3f ",simulationTime);
	}


	//get current and next position key
	PositionKey p0,p1;
	std::vector<PositionKey>::iterator it;
	it=m_positions.begin();
	do { 	//now searching whole list every time, could be optimized
		p0=*it;
		it++;
	} while (it!=m_positions.end() && simulationTime > it->t);
	if(it!=m_positions.end())
		p1=*it;
	else
		p1=p0;

	//interpolate
	PositionKey res;
	double dist=p1.t-p0.t;
	if(dist==0.0)
		res=p0;
	else {
		double delta=simulationTime-p0.t;
		if(delta<0) //if first position key is after current time
			delta=0;
		double a=delta/dist;
		res.t=simulationTime;
		for(int i=0;i<NUM_JOINTS;i++) {
			res.values[i]=(float)(p0.values[i]*(1-a)+p1.values[i]*a);
		}
	}


	unsigned i=0;
	for each(NxRevoluteJoint* j in m_robJoints) {
		float servoPos=res.values[i]; 
		if(debugPrint) fprintf(controllerOut,"%4.1f ",servoPos); //servo position control
		if(servoPos<minMaxAngles[i][0]) servoPos=(float)minMaxAngles[i][0]; //limit checking
		if(servoPos>minMaxAngles[i][1]) servoPos=(float)minMaxAngles[i][1]; //limit checking
		i++;
		float angularPos=servoToAngle(servoPos);

		//angularPos*=lim.high.value*0.9f; //should be limited already
		if(debugPrint) { 
			fprintf(controllerOut,"%.1f ",servoPos); //adjusted for angle limits
			float actualAngle=j->getAngle();
			float actualServoPos=angleToServo(actualAngle);
			fprintf(controllerOut,"%.1f ",actualServoPos); 
		}
		controlRevoluteMotorP(j,angularPos,2.0f,2.5f);
	}
	if(debugPrint) fprintf(controllerOut,"\n");
}

void QuadroMachine::control(float simulationTime)
{
	static FILE* controllerOut=NULL;
	if(PRINT_STATS) {
		if(controllerOut==NULL) {
			controllerOut=fopen("sin_controller_out.txt","w");
			fprintf(controllerOut,"#simtime (jointctrlval(normalized) jointctrlval actualjointangle)x%d xpos ypos absdeltapos\n",m_robJoints.size());
		}
		fprintf(controllerOut,"%.3f ",simulationTime);
	}

	int i=0;
	for each(NxRevoluteJoint* j in m_robJoints) {
		NxJointLimitPairDesc lim;
		j->getLimits(lim); //lim.high and low are equal, initial rotation decided from parameters limLow and limHigh
		float freq=0.05f;
		//float freq=0.2f;
		float phase=m_params->getValue("phase",i); //could also have accumulated phase.. more interesting for other machines?
		float attackTime=m_params->getValue("attack",i);
		float p0Time=m_params->getValue("p0",i);
		float decayTime=m_params->getValue("decay",i);
		float p1Time=m_params->getValue("p1",i);

		float angularPos=calcSinEnvelopeFromParams(attackTime,p0Time,decayTime,p1Time,freq*simulationTime,phase);
		if(PRINT_STATS) fprintf(controllerOut,"%.3f ",angularPos); //normalized controller out

		angularPos*=lim.high.value*0.9f; //should not go all the way out(?)
		if(PRINT_STATS) { 
			fprintf(controllerOut,"%.3f ",angularPos); //adjusted for angle limits
			float actualAngle=j->getAngle();
			fprintf(controllerOut,"%.3f ",actualAngle); 
		}

		controlRevoluteMotorP(j,angularPos,2.0f,2.5f);
		//controlRevoluteMotorP(j,angularPos,10.0f,2.5f); //does not seem to have any effect, probably limited by max speed or force
		
		i++;
	}
}



Part* QuadroMachine::createMachinePart(NxActorDesc& actorDesc, NxActor* attachTo, NxMat34& transform, bool glue)
{
	NxMat34 parentTransform;
	if(attachTo)
		parentTransform=attachTo->getGlobalPose();
	else
		parentTransform.id();

	NxMat34 resTfm;
	resTfm.multiply(parentTransform,transform);

	NxBodyDesc bodyDesc;
	actorDesc.body=&bodyDesc;
	actorDesc.globalPose=resTfm;

	Part *p=new Part(gScene->createActor(actorDesc));
	parts.push_back(p);
	m_robParts.push_back(p);
	if(glue)
		createFixedJoint(attachTo,p->act);
	return p;
}


//mesh loading and transformation
void QuadroMachine::initMeshes()
{
	if(m_pcPartMesh==NULL) { //(static) meshes not loaded yet
		//units in mm from solidworks, scale with 0.1 to work in cm
		//units in mm from solidworks, scale with 0.001 to work in m
		float scale=SC*0.1;
		//the meshes are a bit offset from 0, translate to convenient coordinate  system
		m_pcPartMesh=new MeshGraphicsObject("data/quadrobot/leftsidefitpc",scale,SC*-8.8,SC*-9.4,0);
		m_pcPartMesh->m_mat->setColor(0.5f,0.5f,1);
		m_batteryPartMesh=new MeshGraphicsObject("data/quadrobot/battery_kyrre",scale,SC*-8.8f,0,0,PI,0,0);
		m_batteryPartMesh->m_mat->setColor(0.6f,0.6f,1);
		m_upperLegMesh=new MeshGraphicsObject("data/quadrobot/legintermediate",scale,SC*-3,SC*-2,SC*-4.5f);
		m_upperLegMesh->m_mat->setColor(0.2f,0.2f,1);
		m_lowerLegMesh=new MeshGraphicsObject("data/quadrobot/legending",scale,SC*-1.8f,SC*-0.2f,SC*-2);
		m_lowerLegMesh->m_mat->setColor(0.4f,0.4f,1);
	}
}


NxVec3 QuadroMachine::getPosition()
{
	NxVec3 pos(0,0,0);
	if(getCentralPart()) 
		pos=getCentralPart()->act->getGlobalPosition();
	return pos;
}


Part* QuadroMachine::getCentralPart()
{
	if(m_robParts[0])
		return m_robParts[0];
	else
		return NULL;
}


//consider moving up to machine and rename to more descriptive and more params
void QuadroMachine::controlRevoluteMotorP(NxRevoluteJoint* joint, float targetAngle,float maxSpeed,float proportionalFactor)
{
	//float p=0.1f;
	//float p=2.5f;
	float p=proportionalFactor;
	//float maxSpeed=0.5f;
	float curAngle=joint->getAngle();
	float deltaAngle=targetAngle-curAngle;
	float desiredSpeed=deltaAngle*p;
	desiredSpeed=__min(__max(-maxSpeed,desiredSpeed),maxSpeed); //clamping

	NxMotorDesc motor;
	joint->getMotor(motor);
	motor.velTarget=desiredSpeed;
	joint->setMotor(motor); //automatically enables motor
}

float QuadroMachine::calcSinEnvelopeFromParams(float attackDuration,float p0Duration,float decayDuration,float p1Duration,float t,float phase)
{
	float period=attackDuration+p0Duration+decayDuration+p1Duration;
	attackDuration/=period;
	p0Duration/=period;
	decayDuration/=period;
	p1Duration/=period;
	float t0=0;
	float t1=attackDuration;
	float t2=t1+p0Duration;
	float t3=t2+decayDuration;
	t=__max(t-phase,0); //subtract phase but wait in the beginning so no abrupt movements
	t+=t1/2.0f; //want to start with muscles in neutral position

	float remainder=fmod(t,1.0f);
	return sineEnvelope(remainder,t0,t1,t2,t3)*2-1;
};





//////////////////////////////////////////////////////////////////////////
// EVOLUTION AND FITNESS
//////////////////////////////////////////////////////////////////////////

//should be separated into decodefromGAgenome and calculatefitness (which could be a member function accessible ot other optimisation algorithms)
float calculateQuadroFitness(GAGenome& g)
{
	QuadroEvolutionHarness* evoHarness = (QuadroEvolutionHarness*)g.userData();
	if(!evoHarness->isRunning())
		return 0;
	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	int runFrames=2500;

	QuadroParams p;
	evoHarness->convertParams(g, p);


	QuadroMachine machine(&p);

	NxVec3 firstPos=machine.getPosition();
	NxVec3 lastPos=firstPos;
	double distSum=0.0;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;

	int f;
	for(f=0; f<runFrames && !cheat; f++) {
		bool freeze=false;
		if(glfwGetKey(GLFW_KEY_F3)) freeze=true;
		if( glfwGetKey(GLFW_KEY_ESC) ) {
			evoHarness->stop();
			break; 
		}

		if(freeze && f>0) //freeze hack, ugly
			f--;

		machine.update((float)simTime);
		setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0);
		evoHarness->updateGraphicsAndPhysics(freeze);
		if(!freeze)
			simTime+=evoHarness->m_timestep;
		lastPos=machine.getPosition();

		
		/*for(int i=0;i<3;i++) {
			if(machine.m_touchSensors[i]==true) {
				cheat=true;
				break;
			}
		}*/
	} //end eval loop

	NxVec3 diff=machine.getPosition()-firstPos;
	terminatePhysics();
	float fitnessValue=(float)__max(diff.magnitude(),0); //allows any direction of final movement, not only "forward"
	//float fitnessValue=(float)__max(diff.dot(NxVec3(1,0,0)),0); //only forward
	fitnessValue=fitnessValue*60.0f/runFrames;

	if(cheat) {
		printf(" cheat!");
		fitnessValue=0;
	}
	printf("\t\t\tfit: %f\n",fitnessValue);
	return fitnessValue;
}




QuadroEvolutionHarness::QuadroEvolutionHarness(int seed)
: EvolutionHarness()
{
	setSimulatorTimeStep(1/30.0f);
	GARandomSeed(seed);
	createGA(QuadroParams(),calculateQuadroFitness,"quadrosettings.txt");
}
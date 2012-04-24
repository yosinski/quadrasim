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
#include <boost/foreach.hpp>

#define PRINT_STATS 1 //for printing controller and position stats
#define PRINT_INTERVAL 0.1 //seconds between each log line

//these are class static to speed up loading time
MeshGraphicsObject* QuadroMachine::m_pcPartMesh=NULL;
MeshGraphicsObject* QuadroMachine::m_batteryPartMesh=NULL;
MeshGraphicsObject* QuadroMachine::m_upperLegMesh=NULL;
MeshGraphicsObject* QuadroMachine::m_lowerLegMesh=NULL;

#define SC 1.0f //scale of robot dimensions - originally in cm, sc==0.01 -> physics units are [m]
//nå går den litt gjennom bakken hvis den er for liten - er nødt til å stille inn noe i grafikk (depth buffer)?

//shape constants
float hingeXOffs=-5.2f*SC;
float innerYOffs=3.0f*SC;
float outerYOffs=6.0f*SC;
float innerZOffs=6.2f*SC;
float outerZOffs=3.0f*SC;



Part* QuadroMachine::createPCCore(NxActor* attachTo, NxMat34& transform, bool glue)
{
	//note! coordinate system maybe not as you think!
	NxActorDesc actorDesc;
	//actorDesc.density=1.0f; //this may have to be adjusted
	actorDesc.density=0.3f; //this may have to be adjusted
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
	
	return createMachinePart(actorDesc,attachTo,transform,glue);
}



Part* QuadroMachine::createBatteryCore(NxActor* attachTo, NxMat34& transform, bool glue)
{
	NxActorDesc actorDesc;
	//actorDesc.density=1.0f; //this may have to be adjusted
	actorDesc.density=0.2f; //this may have to be adjusted
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


	//anisotropic friction material
	NxMaterialDesc  material;
	material.restitution        = 0.0f;
	material.staticFriction     = 0.1f;
	material.dynamicFriction    = 0.1f; 
	material.dynamicFrictionV   = 0.8f;
	material.staticFrictionV    = 0.8f;
	material.dirOfAnisotropy.set(0,0,1);
	material.flags              = NX_MF_ANISOTROPIC;
	NxMaterial *anisoMaterial = gScene->createMaterial(material);


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
	b3.materialIndex= anisoMaterial->getMaterialIndex();
	actorDesc.shapes.push_back(&b3);


	return createMachinePart(actorDesc,attachTo,transform,glue);
}

Part* QuadroMachine::createLeg(NxActor* attachTo, NxMat34& transform, bool glue)
{
	Part* lastPart=NULL;
	Part* curPart=NULL;

	//upper leg part and joint to body
	curPart=createUpperLeg(attachTo,transform,glue);
	if(attachTo) {
		createMotorJoint(curPart->act,NxVec3(0,0,1),attachTo,m_innerAngleMin,m_innerAngleMax);
	}
	lastPart=curPart;

	//lower leg and joint to upper leg
	NxMat34 temp = createTfm(SC*-0.4f,SC*10,SC*0,0,0,0);
	curPart=createLowerLeg(lastPart->act,temp,false);
	createMotorJoint(curPart->act,NxVec3(0,0,1),lastPart->act,m_outerAngleMin,m_outerAngleMax);
	return lastPart;
}


//think if any of this should be moved to parent quadrobot class
QuadroMachine::QuadroMachine(QuadroParams* params/* =NULL */)
{
	init(params); //quadrobot common init

	initMeshes();

	Part* lastPart=NULL;
	Part* curPart=NULL;

	//trodde noen av joints var feil retning pga. flippede akser mm i debug render
	//alt ok etter manuell testing. kan hende lokale akser byttes men fungerer allikevel.
	//core joint [orientering min/maks] bï¿½r sjekkes mot hvordan roboten er satt opp.

	//New order, pc and battery parts were wrong wrt. leg order
	NxMat34 temp1 = createTfm(0,SC*40,0,0,0,PI/2);
	Part* batteryPart=lastPart=createBatteryCore(NULL,temp1,false);
	NxMat34 temp6 = createTfm(SC*-5.5f,SC*7,SC*6.4f,PI/4,PI,0);
	createLeg(lastPart->act,temp6,false);
	NxMat34 temp2 = createTfm(SC*-5.5f,SC*-7,SC*6.4f,3*PI/4,PI,0);
	createLeg(lastPart->act,temp2,false);
	//other side with legs
	NxMat34 temp3 = createTfm(0,0,0,PI,0,0);
	Part* pcPart=curPart=createPCCore(lastPart->act,temp3,false);
	lastPart=curPart;
	NxMat34 temp4 = createTfm(SC*-5.5f,SC*7,SC*6.4f,PI/4,PI,0);
	createLeg(lastPart->act,temp4,false);
	NxMat34 temp5 = createTfm(SC*-5.5f,SC*-7,SC*6.4f,3*PI/4,PI,0);
	createLeg(lastPart->act,temp5,false);
	//add joint between pc and battery
	createMotorJoint(pcPart->act,NxVec3(0,0,1),batteryPart->act,m_coreAngleMin,m_coreAngleMax);

	//can add sensors here
	//can increase solver accuracy here
	//int solverIterations=100; //4 is default
	int solverIterations=200; //4 is default
	BOOST_FOREACH(Part* part, m_robParts) //syntax not portable
		part->act->setSolverIterationCount(solverIterations);


	//experiment:
	/*printf("Actor masses:\n");
	for(int i=0;i<m_robParts.size();i++) {
		float mass=m_robParts[i]->act->getMass();
		printf("mass of part %d: %.2f\n",i,mass);
	}*/


	simLogFile=NULL;
	logSim=false;
}


NxRevoluteJoint* QuadroMachine::createMotorJoint(NxActor* p1,NxVec3 p1LocalAxis,NxActor* p2,float minAngle,float maxAngle)
{
	//float maxForce=1000000;
	float maxForce  =85000; //old value
	//float maxForce  =100000; //old value
	NxRevoluteJoint* joint;
	NxVec3 globalAxis;
	p1->getGlobalOrientation().multiply(p1LocalAxis,globalAxis);
	joint=createRevoluteMotorJoint(p1,p2,p1->getGlobalPose().t,globalAxis,minAngle,maxAngle,maxForce);

	m_robJoints.push_back(joint);
	return joint;
}



void QuadroMachine::update(float simulationTime)
{

	updateControl(simulationTime);

	if(logSim && simLogFile) fprintf(simLogFile,"%.3f ",simulationTime); 

	unsigned i=0;
	BOOST_FOREACH(NxRevoluteJoint* j, m_robJoints) {
		float angularPos=m_jointTargets[i];

		//check limits here or elsewhere?
		NxJointLimitPairDesc lims;
		j->getLimits(lims);
		if(angularPos<lims.low.value) angularPos=lims.low.value; //limit checking
		if(angularPos>lims.high.value) angularPos=lims.high.value; //limit checking
		//controlRevoluteMotorP(j,angularPos,2.0f,2.5f);
		controlRevoluteMotorP(j,angularPos,2.5f,20.0f);

		if(logSim && simLogFile) fprintf(simLogFile,"%.1f ",angleToServo(angularPos)); //log target position (in servo coordinates)
		if(logSim && simLogFile) fprintf(simLogFile,"%.1f ",angleToServo(j->getAngle())); //log actual position (in servo coordinates)

		i++;
	}

	if(logSim && simLogFile) {
		NxVec3 pos=getPosition();
		fprintf(simLogFile,"%.2f %.2f %.2f",pos.x,pos.y,pos.z); //log actual position (in servo coordinates)
		fprintf(simLogFile,"\n",simulationTime); 
		/*float dist=(pos-NxVec3(0,0,0)).magnitude(); //assuming it started in 0,0,0
		float speed=dist/simulationTime;
		printf("time: %.2f distance from 0: %.2f speed: %.2f\n",simulationTime,dist,speed);*/
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
	float curAngle=joint->getAngle();
	float deltaAngle=targetAngle-curAngle;
	float desiredSpeed=deltaAngle*p;
	desiredSpeed=fmin(fmax(-maxSpeed,desiredSpeed),maxSpeed); //clamping

	NxMotorDesc motor;
	joint->getMotor(motor);
	motor.velTarget=desiredSpeed;
	joint->setMotor(motor); //automatically enables motor
}


void QuadroMachine::enableSimLogging(bool enable)
{
	//later: add option for logging on specific frames - auto-switch off ?
	std::string logFileName="log_sim.txt";
	logSim=enable;
	if(simLogFile==NULL) {
		printf("opening log file %s for writing\n",logFileName.c_str());
		if(!(simLogFile=fopen(logFileName.c_str(),"w"))) {
			printf("could not open log file %s for writing\n",logFileName.c_str());
			logSim=false;
		}
	}
	fprintf(simLogFile,"# simtime (joint_commanded joint_actual_pos)x%d pos.x pos.y pos.z\n",NUM_JOINTS);
}




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

	gScene->setGravity(NxVec3(0,-9.81*100.0f,0)); //depending on scale

	//int runFrames=2500;
	float evalDuration=7.0f; //seconds
	//float evalDuration=15.0f; //seconds
	int runFrames=(int)(evalDuration/QUADRO_TIMESTEP);

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
	float fitnessValue=fmax(diff.magnitude(),0); //allows any direction of final movement, not only "forward"
	//float fitnessValue=(float)__max(diff.dot(NxVec3(1,0,0)),0); //only forward
	//fitnessValue=fitnessValue/runFrames/QUADRO_TIMESTEP;
	fitnessValue=fitnessValue/evalDuration; //should give cm/s

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
	setSimulatorTimeStep(QUADRO_TIMESTEP);
	GARandomSeed(seed);
	createGA(QuadroParams(),calculateQuadroFitness,"settings_quadro.txt");
}


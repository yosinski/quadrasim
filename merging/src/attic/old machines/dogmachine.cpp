#include "dogmachine.h"
#include "../physx/MyCloth.h"

//for evolution:
#include "../evolutionharness.h"
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#include "../physics.h"
#include "../graphics/graphics.h"
#include "../base/system.h"
#define cout STD_COUT


//hvordan skal man egentlig håndtere store genomer nå? tungvint å ha navn på alt? 
// -gjøre det automatisk med navn/strings?
// -ha en mellomstruktur med strings / interface?
// -?


DogMachine::DogMachine(DogParams& params)
{
	debugFile=fopen("controller_out.txt","w");

	for(int i=0;i<8;i++)
		muscles[i]=NULL;
	for(int i=0;i<4;i++)
		touchSensors[i]=true; //pretend like on ground in beginning
	Part *leg1,*leg2,*leg3,*leg4;

	float leg1Len=params.paramList["leg1len"].val;
	float leg2Len=leg1Len;
	//float legDensity=0.15f; //denne funket for gaits
	float legDensity=0.25f;
	float legWidth=0.5f;
	float footLen=1.0f;

	//platform
	float platThickness=0.7f;
	//float platHeight=__max(leg1Len,leg2Len)*2+2;
	//float platHeight=__max(leg1Len,leg2Len)*2+platThickness+1+0.5f; //add some margin..
	float platHeight=__max(leg1Len,leg2Len)*2+platThickness+1+footLen+0.2f; //add some margin..
	//float platHeight=__max(leg1Len,leg2Len)*2+platThickness+1+footLen*2+1+0.5f;
	//float platDensity=0.05f; //works for gaits
	float platDensity=0.25f;
	float depth=6;
	float platLen=params.paramList["platlen"].val;
	topPart=createBoxPart(Point3D(0,platHeight,0),Point3D(platLen,platThickness,depth));
	topPart->act->updateMassFromShapes(platDensity,0);
	//topPart->physicsEquivalent->updateMassFromShapes(0,platMass);

	//legs    - change to capsules?
	float z=depth-1;
	float legFromEdge=params.paramList["legfromedge"].val; //evt ha separat foran og bak
	leg1=createCapsulePart(Point3D(platLen*legFromEdge,platHeight-leg1Len-1-platThickness,z),leg1Len*2,true,legWidth);
	leg2=createCapsulePart(Point3D(-platLen*legFromEdge,platHeight-leg2Len-1-platThickness,z),leg2Len*2,true,legWidth);
	leg3=createCapsulePart(Point3D(platLen*legFromEdge,platHeight-leg1Len-1-platThickness,-z),leg1Len*2,true,legWidth);
	leg4=createCapsulePart(Point3D(-platLen*legFromEdge,platHeight-leg2Len-1-platThickness,-z),leg2Len*2,true,legWidth);

	
	//experiment: feet again
	NxCapsuleShapeDesc cd;
	cd.radius=legWidth;
	cd.height=footLen;
	cd.localPose.t.set(cd.height/2.0f,-leg1Len-cd.height/2.0f,0);
	cd.localPose.M.rotZ(PI/3.0f);
	leg1->act->createShape(cd);
	leg2->act->createShape(cd);
	leg3->act->createShape(cd);
	leg4->act->createShape(cd);



	//this is tailored to capsule legs
	//float sensorSz=0.5f;
	//leg1->addTouchSensor(NxVec3(0,-leg1Len-legWidth,0),&touchSensors[0],sensorSz);
	//leg2->addTouchSensor(NxVec3(0,-leg2Len-legWidth,0),&touchSensors[1],sensorSz);
	//leg3->addTouchSensor(NxVec3(0,-leg1Len-legWidth,0),&touchSensors[2],sensorSz);
	//leg4->addTouchSensor(NxVec3(0,-leg2Len-legWidth,0),&touchSensors[3],sensorSz);
	float sensorSz=0.5f;
	leg1->addTouchSensor(NxVec3(footLen,-leg1Len-legWidth-footLen/2,0),&touchSensors[0],sensorSz);
	leg2->addTouchSensor(NxVec3(footLen,-leg2Len-legWidth-footLen/2,0),&touchSensors[1],sensorSz);
	leg3->addTouchSensor(NxVec3(footLen,-leg1Len-legWidth-footLen/2,0),&touchSensors[2],sensorSz);
	leg4->addTouchSensor(NxVec3(footLen,-leg2Len-legWidth-footLen/2,0),&touchSensors[3],sensorSz);



	//adjusting leg mass/density for stable system
	leg1->act->updateMassFromShapes(legDensity,0);
	leg2->act->updateMassFromShapes(legDensity,0);
	leg3->act->updateMassFromShapes(legDensity,0);
	leg4->act->updateMassFromShapes(legDensity,0);



	//joints attaching legs to top part
	float limLow=-0.35f*PI;
	float limHigh=0.35f*PI;
	//createRevoluteJoint(topPart->physicsEquivalent,leg1->physicsEquivalent,NxVec3(platLen*legFromEdge,platHeight-platThickness-0.5f,z),NxVec3(0,0,1),limLow,limHigh);
	//createRevoluteJoint(topPart->physicsEquivalent,leg2->physicsEquivalent,NxVec3(-platLen*legFromEdge,platHeight-platThickness-0.5f,z),NxVec3(0,0,1),limLow,limHigh);
	//createRevoluteJoint(topPart->physicsEquivalent,leg3->physicsEquivalent,NxVec3(platLen*legFromEdge,platHeight-platThickness-0.5f,-z),NxVec3(0,0,1),limLow,limHigh);
	//createRevoluteJoint(topPart->physicsEquivalent,leg4->physicsEquivalent,NxVec3(-platLen*legFromEdge,platHeight-platThickness-0.5f,-z),NxVec3(0,0,1),limLow,limHigh);
	//nå er det litt vinkler på beina...
	//createRevoluteJoint(topPart->physicsEquivalent,leg1->physicsEquivalent,NxVec3(platLen*legFromEdge,platHeight-platThickness-0.5f,z),NxVec3(-0.3f,0,1),limLow,limHigh);
	//createRevoluteJoint(topPart->physicsEquivalent,leg2->physicsEquivalent,NxVec3(-platLen*legFromEdge,platHeight-platThickness-0.5f,z),NxVec3(0.3f,0,1),limLow,limHigh);
	//createRevoluteJoint(topPart->physicsEquivalent,leg3->physicsEquivalent,NxVec3(platLen*legFromEdge,platHeight-platThickness-0.5f,-z),NxVec3(0.3f,0,1),limLow,limHigh);
	//createRevoluteJoint(topPart->physicsEquivalent,leg4->physicsEquivalent,NxVec3(-platLen*legFromEdge,platHeight-platThickness-0.5f,-z),NxVec3(-0.3f,0,1),limLow,limHigh);
	createRevoluteJoint(topPart->act,leg1->act,NxVec3(platLen*legFromEdge,platHeight-platThickness-0.5f,z),NxVec3(-0.3f,0,1),limLow,limHigh);
	createRevoluteJoint(topPart->act,leg2->act,NxVec3(-platLen*legFromEdge,platHeight-platThickness-0.5f,z),NxVec3(-0.3f,0,1),limLow,limHigh);
	createRevoluteJoint(topPart->act,leg3->act,NxVec3(platLen*legFromEdge,platHeight-platThickness-0.5f,-z),NxVec3(0.3f,0,1),limLow,limHigh);
	createRevoluteJoint(topPart->act,leg4->act,NxVec3(-platLen*legFromEdge,platHeight-platThickness-0.5f,-z),NxVec3(0.3f,0,1),limLow,limHigh);




	//4 muscles each side, but symmetrical attachments
	//int heightDivs=6;
	int heightDivs=8;
	//int aroundDivs=40;
	int aroundDivs=32;
	int endPoint=(heightDivs+1)*aroundDivs-1;
	//float musWidth=1.0f;
	//float musWidth=1.25f;
	float musWidth=params.paramList["muscleWidth"].val;
	float attachHighX,attachHighY,attachLow;

	//create two sets of muscles (different z)
	for(int i=0;i<=4;i+=4) {
		//mus1 in front of front leg
		attachHighY=platHeight+platThickness;
		attachHighX=platLen*0.95f;
		attachLow=platHeight-platThickness-1-leg1Len-params.paramList["leg1attlofront"].val*leg1Len;
		muscles[i+0]=createMuscle(NxVec3(platLen*legFromEdge,attachLow,z),NxVec3(attachHighX,attachHighY,z),musWidth,heightDivs,aroundDivs);
		muscles[i+0]->getNxCloth()->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);

		//mus2 behind front leg
		attachLow=platHeight-platThickness-1-leg1Len-params.paramList["leg1attloback"].val*leg1Len;;
		attachHighX=params.paramList["leg1atthiback"].val*platLen*legFromEdge*0.9f;
		muscles[i+1]=createMuscle(NxVec3(platLen*legFromEdge,attachLow,z),NxVec3(attachHighX,attachHighY,z),musWidth,heightDivs,aroundDivs);
		muscles[i+1]->getNxCloth()->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);

		//mus3 front of back leg
		attachLow=platHeight-platThickness-1-leg2Len-params.paramList["leg2attlofront"].val*leg2Len;
		attachHighX=params.paramList["leg2atthifront"].val*platLen*legFromEdge*0.9f; //"in front" now
		muscles[i+2]=createMuscle(NxVec3(-platLen*legFromEdge,attachLow,z),NxVec3(-attachHighX,attachHighY,z),musWidth,heightDivs,aroundDivs);
		muscles[i+2]->getNxCloth()->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);

		//mus4 behing rear leg
		attachLow=platHeight-platThickness-1-leg2Len-params.paramList["leg2attloback"].val*leg2Len;
		attachHighX=platLen*0.95f;
		muscles[i+3]=createMuscle(NxVec3(-platLen*legFromEdge,attachLow,z),NxVec3(-attachHighX,attachHighY,z),musWidth,heightDivs,aroundDivs);
		muscles[i+3]->getNxCloth()->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);

		z=-z;
	}


	//control params
	m_p0=params.paramList["pause0"].val;
	m_frequency=params.paramList["freq"].val;
	m_phase0=params.paramList["phase0"].val;
	m_phase1=params.paramList["phase1"].val;
	m_attack=params.paramList["attack"].val;
	m_decay=params.paramList["decay"].val;


}




void DogMachine::update(float simulationTime)
{
	float maxPressure=3.0f;
	//float minPressure=1.1f;
	float minPressure=0.7f;
	//envelope timing
	//t0-t1: up
	//t1-t2: high
	//t2-t3: down
	//t4-t4: low
	float t0=0.0f;
	//float t1=0.1f;
	float t1=m_attack;
	float t2=t1+m_p0;
	//float t3=t2+0.1f;
	float t3=t2+m_decay;
	if(t3>0.98f) 
		t3=0.98f; //must have some down time
	float t4=1.0f;
	float delta=1.0f/(t1-t0); //rise/fall speed

	//er det kanskje noe feil her ifht offsets?

	//envelope calculation
	float t=m_frequency*simulationTime;

	float remainder=fmod(t,1.0f);
	float pressure=linearEnvelope(remainder,t0,t1,t2,t3);
	fprintf(debugFile,"%.2f ",pressure);
	muscles[0]->getNxCloth()->setPressure(pressure*maxPressure+minPressure);
	muscles[1]->getNxCloth()->setPressure((1.0f-pressure)*maxPressure+minPressure);

	remainder=fmod(t+m_phase0,1.0f);
	pressure=linearEnvelope(remainder,t0,t1,t2,t3);
	fprintf(debugFile,"%.2f ",pressure);
	muscles[2]->getNxCloth()->setPressure(pressure*maxPressure+minPressure);
	muscles[3]->getNxCloth()->setPressure((1.0f-pressure)*maxPressure+minPressure);

	remainder=fmod(t+m_phase1,1.0f);
	pressure=linearEnvelope(remainder,t0,t1,t2,t3);
	fprintf(debugFile,"%.2f ",pressure);
	muscles[4]->getNxCloth()->setPressure(pressure*maxPressure+1.0f);
	muscles[5]->getNxCloth()->setPressure((1.0f-pressure)*maxPressure+1.0f);

	remainder=fmod(t+m_phase1+m_phase0,1.0f);
	pressure=linearEnvelope(remainder,t0,t1,t2,t3);
	fprintf(debugFile,"%.2f ",pressure);
	muscles[6]->getNxCloth()->setPressure(pressure*maxPressure+1.0f);
	muscles[7]->getNxCloth()->setPressure((1.0f-pressure)*maxPressure+1.0f);

	fprintf(debugFile,"\n");

}


MyCloth* DogMachine::createMuscle(NxVec3& startPos,NxVec3& endPos,float radius,int heightDivs,int aroundDivs)
{

	NxClothDesc clothDesc;
	clothDesc.globalPose.t = startPos;

	//find orientation of muscle, is there an easier way?
	//now calculating axis-angle information
	NxVec3 dir=endPos-startPos;
	dir.normalize();
	NxVec3 up(0,1,0); //"up" vetcctor as starting point
	NxVec3 axis;
	if(dir.equals(up,0.0001f))
		axis.set(0,0,1);
	else
		axis=up.cross(dir); //using "up" vector  

	float angle=acos(up.dot(dir))/PI*180.0f;
	clothDesc.globalPose.M.fromQuat(NxQuat(angle,axis));
	
	
	clothDesc.hierarchicalSolverIterations=3; //default: 2
	//clothDesc.dampingCoefficient = 0.0f; //default:0.5
	clothDesc.density=1.0f;
	clothDesc.friction = 0.5f;
	clothDesc.pressure = 1.0f;
	//clothDesc.solverIterations = 8; //default: 5
	//clothDesc.solverIterations = 10; //default: 5
	clothDesc.attachmentResponseCoefficient=1.0f;
	//clothDesc.bendingStifness=0.8f; //default: 1 (very stiff)  close to 0: easy to bend
	//clothDesc.stretchingStiffness=0.7f;  //1 is very difficult to stretch, 0 not allowed but close to 0 easy to stretch. 1 was default (?)
	clothDesc.flags |= NX_CLF_PRESSURE;
	//clothDesc.flags |= NX_CLF_GRAVITY;
	clothDesc.flags |= NX_CLF_BENDING;
	clothDesc.flags |= NX_CLF_BENDING_ORTHO; //more realistic
	clothDesc.flags |= NX_CLF_DAMPING;
	clothDesc.flags |= NX_CLF_COMDAMPING;
	clothDesc.flags |= NX_CLF_COLLISION_TWOWAY; //cloth can also pull other things
	clothDesc.flags |= NX_CLF_DISABLE_COLLISION; //collision with other objects.. can still pull objects

	if(gHardwareSimulation)
		clothDesc.flags |= NX_CLF_HARDWARE;

	float height=startPos.distance(endPos);
	MyCloth *objCloth = new MyCloth(gScene,clothDesc,height,radius,heightDivs,aroundDivs); //use handmade muscle constructor
	cloths.push_back(objCloth);
	return objCloth;
}







//////////////////////////////////////////////////////////////////////////////////

///evodog stuff

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
	GABin2DecGenome* ind=(GABin2DecGenome*)&g;
	ind->GA1DBinaryStringGenome::write(cout);
	cout << std::endl;

	DogMachine machine(p);

	int touchCount[4]={0};

	NxVec3 firstPos=machine.topPart->act->getGlobalPose().t;
	NxVec3 lastPos=firstPos;
	float highestPos=firstPos.y;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;
	int runFrames=800; 
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
		evoHarness->updateGraphicsAndPhysics(freeze);
		if(!freeze)
			simTime+=evoHarness->m_timestep;

		lastPos=machine.topPart->act->getGlobalPose().t;

		//try to detect jumping/floating
		float y=machine.topPart->act->getGlobalPose().t.y;
		if(y > firstPos.y*1.2f || y < firstPos.y*0.5f) {
			cheat=true;
			break;
		}
		
		//try also to detect hovering in some way
		int count=0;
		for(int i=0;i<4;i++)
			if(!machine.touchSensors[i])
				count++;
		/*if(count>=3) {
			printf(">=3 legs in air ");
			cheat=true;
			break;
		}*/

		for(int i=0;i<4;i++)
			if(machine.touchSensors[i])
				touchCount[i]++;
			
	}//end eval loop
	NxVec3 diff=machine.topPart->act->getGlobalPose().t-firstPos;
	terminatePhysics();
	float fitnessValue=diff.magnitude();
	
	int dragCount=0;
	for(int i=0;i<4;i++) {
		float amount=(float)touchCount[i]/runFrames;
		printf("%.2f ",amount);
		if(amount > 0.98f)
			dragCount++;
	}
	printf("\n");
	/*if(dragCount>=3) {
		printf("dragging 3 or more legs or standing in place -> ");
		cheat=true;
	}*/


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
		//mapping.add(8, it->second.minVal, it->second.maxVal); //8b precision
		mapping.add(it->second.precision, it->second.minVal, it->second.maxVal); //specified precision
	}

	GABin2DecGenome genome(mapping, calculateDogFitness, (void *)this); //pass a pointer to this object
	GAPopulation pop(genome);
	pop.evaluator(forcedPopEvaluator); //this uses the custom evaluator which forces the elite to be evaluated each time
	//m_ga = new GASimpleGA(genome);
	m_ga = new GASimpleGA(pop);
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

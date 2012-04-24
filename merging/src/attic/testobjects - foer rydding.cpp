#include "physics.h"
#include "graphics/graphics.h"
#include "testobjects.h"
#include "graphics/MeshGraphicsObject.h"
#include "base/system.h"
#include "physx/MyCloth.h"
#include "graphics/glstuff.h"
#include "mocap.h"

#include "machines/stdpartrobot.h"
#include "machines/dogmachine.h"
#include "machines/legmachine.h"
//#include "machines/x2machine.h"
#include "machines/x2industrial.h"
#include "machines/x2crawler.h"
#include "machines/x2industrial2.h"
#include "machines/quadromachine.h"
#include "machines/ClimbingFish.h"


MyCloth* mus1=NULL;
MyCloth* mus2=NULL;
MyCloth* mus3=NULL;
void updateMuscleSetup2();

//later: make a list of machines (with virtual update function), no need to reference specific ones here
DogMachine* dog=NULL;
LegMachine* leg=NULL;
X2Industrial* x2=NULL;
X2Crawler* crawler=NULL;
X2Industrial2* industrial2=NULL;
QuadroMachine* quadro=NULL;
ClimbingFish* climbingFish=NULL;

//hack for follow cam
Part* followPart=NULL;

int frameCount=0; //hacky hack

ImageRecorder* imgRecorder=NULL;

//mocap test
std::vector<Part*> markerParts;
Mocap* mocap=NULL;
std::vector<Part*> bodyParts;

	
struct SnakyThing
{
	std::vector<Part*> controlParts;
	static const int numSegs=10;
	static const int numThreads=10;

	Part* createChain(NxVec3 startPos) {
		float length=2.0;
		Part *lastPart;
		float x,y,z;
		x=y=z=0;
		Part *p0=createCapsulePart(startPos+NxVec3(0,0,0),length,false);
		lastPart=p0;

		for(int seg=1;seg<=numSegs;seg++) {
			y=(length+2)*seg;
			Part *p=createCapsulePart(startPos+NxVec3(x,y,0),length,false);
			joints.push_back(createRevoluteJoint(lastPart->act,p->act,startPos+NxVec3(x,y,z),NxVec3(0,0,1)));
			lastPart=p;
		}
		return p0;
	}

	SnakyThing() 
	{
		Part* lastPart=NULL;
		for(int t=0;t<numThreads;t++) {
			Part* p=createChain(NxVec3(t*3.0f,0,0));
			//controlParts.push_back(p);
			if(lastPart)
				createRevoluteJoint(lastPart->act,p->act,NxVec3(t*3.0f,0,0),NxVec3(0,0,1));
				//createFixedJoint(lastPart->act,p->act);
			lastPart=p;
		}
	}

};
SnakyThing* thingy=NULL;


//temp hack func
void cleanUpTestObjects()
{
	mus1=mus2=mus3=NULL;
	delete dog;
	delete leg;
	dog=NULL;
	leg=NULL;
	delete x2;
	x2=NULL;
	delete crawler;
	crawler=NULL;
	delete industrial2;
	industrial2=NULL;
	followPart=NULL;
	delete thingy;
	thingy=NULL;
	delete quadro;
	quadro=NULL;
	delete climbingFish;
	climbingFish=NULL;

	markerParts.resize(0);
	bodyParts.resize(0);
}

void updateTestObjects()
{
	updateMuscleSetup2();
}

MyCloth* generateMuscle(NxVec3& pos, float height,float radius,int heightDivs,int aroundDivs)
{
	NxClothDesc clothDesc;
	clothDesc.globalPose.t = pos;
	clothDesc.dampingCoefficient = 0.7f; //hva er mye? 1 eller 0?
	//clothDesc.density=5.0f;
	clothDesc.friction = 0.5f;
	clothDesc.pressure = 1.0f;
	clothDesc.solverIterations = 10; //default: 5
	//clothDesc.solverIterations = 8; //default: 5
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



void updateMuscleSetup2()
{
	if(!mus1 || !mus2 || !mus3) return;

	static int frameCount=0; //hack
	frameCount++;
	float maxPressure=3.0f;
	//float pressure=(sin(frameCount/20.0f)*0.5f+0.5f)*3.0f + 1.0f; //1-4 atm
	float p=(sin(frameCount/20.0f)*0.5f+0.5f);
	mus1->getNxCloth()->setPressure(p*maxPressure+1);
	mus2->getNxCloth()->setPressure(p*maxPressure+1);
	mus3->getNxCloth()->setPressure(p*maxPressure+1);
}



void createMuscleSetup2()
{
	float startHeight=4.0f;
	float height=10;
	int heightDivs=8;
	int aroundDivs=16;
	float cubeSize=0.6f;
	int endPoint=(heightDivs+1)*aroundDivs-1;
	float xOffs=-8.0f;
	Part* cube;
	
	mus1=generateMuscle(NxVec3(xOffs,startHeight,0),height,1.5f,heightDivs,aroundDivs);
	mus1->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+height,0));
	cube=createBoxPart(Point3D(xOffs,startHeight-cubeSize,0),Point3D(cubeSize,cubeSize,cubeSize));
	cube->act->setMass(0.6f); //for tung last kan ødelegge..
	mus1->getNxCloth()->attachToShape(cube->act->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);


	heightDivs=32;
	aroundDivs=32;
	xOffs=0;
	mus2=generateMuscle(NxVec3(xOffs,startHeight,0),height,1.5f,heightDivs,aroundDivs);
	endPoint=(heightDivs+1)*aroundDivs-1;
	mus2->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+height,0));
	cube=createBoxPart(Point3D(xOffs,startHeight-cubeSize+0.1f,0),Point3D(cubeSize,cubeSize,cubeSize));
	cube->act->setMass(0.6f); //for tung last kan ødelegge..
	mus2->getNxCloth()->attachToShape(cube->act->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);

	heightDivs=8;
	aroundDivs=64;
	xOffs=8;
	mus3=generateMuscle(NxVec3(xOffs,startHeight,0),height,1.5f,heightDivs,aroundDivs);
	endPoint=(heightDivs+1)*aroundDivs-1;
	mus3->getNxCloth()->attachVertexToGlobalPosition(endPoint,NxVec3(xOffs,startHeight+height,0));
	cube=createBoxPart(Point3D(xOffs,startHeight-cubeSize+0.1f,0),Point3D(cubeSize,cubeSize,cubeSize));
	cube->act->setMass(0.6f); //for tung last kan ødelegge..
	mus3->getNxCloth()->attachToShape(cube->act->getShapes()[0],NX_CLOTH_ATTACHMENT_TWOWAY);
}






void createSnake()
{
	float length=2.0;
	Part *lastPart;
	float x,y,z;
	x=y=z=0;

	//length of one capsule = length + 2 * rad(=1)
	Part *p0=createCapsulePart(Point3D(0,0,0),length,false);
	lastPart=p0;

	for(int seg=1;seg<10;seg++) {
		y=(length+2)*seg;
		Part *p=createCapsulePart(Point3D(0,y,0),length,false);

		NxRevoluteJoint *j=createRevoluteJoint(lastPart->act,p->act,NxVec3(x,y,z),NxVec3(0,0,1));
		if(j==NULL) systemError("funka ikke");
		joints.push_back(j);
		lastPart=p;
	}
}



/*void createMocapBodyTest()
{
	mocap::getTorsoPosition;
	...getTorsoOrientation
	etc..

	lag bokser her
	kalkuleringer i mocap->cpp
}*/

void createMocapTest()
{
	if(mocap==NULL) {
		mocap=new Mocap;
		//mocap->loadFromFile("mocap_processed.txt");
		mocap->loadFromFile("mocap_intro.txt");
		mocap->processDataForIntro();
		
		//mocap->scaleData(0.1f);
	}
	float partSz=2.0f;

 	//markerParts.resize(0);
	for(int i=0;i<mocap->m_nMarkers;i++) {
		Vec3 pos=mocap->getPosition(0,i);
		markerParts.push_back(createBoxPart(Point3D(pos.x,pos.y,pos.z),Point3D(partSz,partSz,partSz),false));
	}

	//createMocapBodyTest();
}


//move the simulation steps to the main loop and the motor updates to individual functions for each test object
void animateAllJoints()
{
	//static int frameCount=0; //hacky hack
	if(!gFreeze) 
		frameCount++; //hacky hack

	//experi
	if(markerParts.size()>0) {
		for(int i=0;i<mocap->m_nMarkers;i++) {
			NxActor* act=markerParts[i]->act;
			Vec3 pos=mocap->getPosition(fmod(frameCount/60.0f,mocap->m_endTime),i);
			act->setGlobalPosition(NxVec3(pos.x,pos.y,pos.z));
		}
	}

	//lag heller en egen funksjon for hvert objekt som oppdaterer
	for(unsigned j=0;j<joints.size();j++) {
		NxMotorDesc mDesc;
		mDesc.maxForce=1000;
		float mSpeed;
		if(sin((frameCount+j*10)/10.0)>0) mSpeed=-3;
		else mSpeed=3;
		mDesc.velTarget=mSpeed;
		joints[j]->setMotor(mDesc);
	}

	if(followPart) {
		//setFollowTargetAndPanning(Vec3((float*)(followPart->act->getGlobalPosition()+NxVec3(0,20,0)).get()),frameCount*0.0005f);
		//setFollowTargetAndPanning(Vec3((float*)(followPart->act->getGlobalPosition()+NxVec3(0,20,0)).get()),frameCount*0.0002f);
		setFollowTargetAndPanning(Vec3((float*)(followPart->act->getGlobalPosition()+NxVec3(0,10,0)).get()),0);

	}

	if(dog) 
		dog->update(frameCount/60.0f);
	if(leg) 
		leg->update(frameCount/60.0f);
	if(x2) 
		x2->update(frameCount/60.0f);
	if(crawler)
		crawler->update(frameCount/60.0f);
	if(industrial2) 
		industrial2->update(frameCount/60.0f);
	if(quadro)
		quadro->update(frameCount/60.0f);
	if(climbingFish)
		climbingFish->update(frameCount/60.0f);
}



/*bool somethingTouched=false;
void createSensorTestObject()
{
	Part *p=createCapsulePart(Point3D(0,20,0));
	NxSphereShapeDesc trig;
	trig.localPose.t.set(0,-1.5f,0);
	trig.radius=0.2f;
	trig.shapeFlags |= NX_TRIGGER_ENABLE;
	trig.userData=&somethingTouched;
	p->physicsEquivalent->createShape(trig);
}*/




void setupTestObjectScene()
{
	cleanUpTestObjects();
	terminatePhysics();
	initPhysics(); //hvor mye av dette må gjøres for reset??
	frameCount=0;
}


void testObjectLoop()
{
	setupTestObjectScene();
	//imgRecorder=new ImageRecorder(20,"d:/temp/video_out/testvideo",90);
	//imgRecorder->start(); //could move to keyboard activation
	static double lastTime=glfwGetTime();
	bool animateJoints=false;
	bool running = true;
	while( running )
	{
		updateFPS();
		updateWindowSize();

		//selection of test objects
		Point3D pos(randRangef(-1,1)*12,40.0f, randRangef(-1,1)*12);
		Point3D dims(randRangef(0.2f,2.0f),randRangef(0.2f,2.0f),randRangef(0.2f,2.0f));
		if(glfwGetKey( GLFW_KEY_SPACE)) createBoxPart(pos,dims);
		if(glfwGetKey( GLFW_KEY_ENTER)) createSpherePart(pos,dims.x);
		if(glfwGetKey( GLFW_KEY_LSHIFT)) createCapsulePart(pos,dims.x*2,true,dims.y);
		double curTime=glfwGetTime();
		if(curTime-lastTime > 0.1) {
			if(glfwGetKey( GLFW_KEY_RSHIFT)) createSnake();
			if(glfwGetKey( GLFW_KEY_TAB)) createRobot();
			if(glfwGetKey('1')) generateMuscle(NxVec3(0,20,0),10,2,8,20);
			if(glfwGetKey('2')) createMuscleSetup2();
			if(glfwGetKey('-')) {
				if(thingy==NULL)
					thingy=new SnakyThing();
			}
			if(glfwGetKey('5'))  {
				if(climbingFish==NULL) {
					ClimbingFishParams p;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p.loadFromFile("bestind.txt"))
							systemError("could not load climbing fish params");
					climbingFish=new ClimbingFish(p);
				}
			}
			if(glfwGetKey('6'))  {
				if(dog==NULL) {
					DogParams p;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p.loadFromFile("bestind.txt"))
							systemError("could not load dog params");
					dog=new DogMachine(p);
				}
			}
			if(glfwGetKey('7'))  {
				if(leg==NULL) {
					LegParams p;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p.loadFromFile("bestind.txt"))
							systemError("could not load leg params");
					leg=new LegMachine(p,LegMachine::CLOTH_MUSCLE);
				}
			}
			if(glfwGetKey('8'))
				if(leg==NULL) {
					LegParams p;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p.loadFromFile("bestind.txt"))
							systemError("could not load leg params");
					leg=new LegMachine(p,LegMachine::SPRING_MUSCLE);
				}
			if(glfwGetKey('9'))
				if(leg==NULL) {
					LegParams p;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p.loadFromFile("bestind.txt"))
							systemError("could not load leg params");
					leg=new LegMachine(p,LegMachine::DISTANCE_MUSCLE);
				}
			if(glfwGetKey('0'))
				createMocapTest();

			if(glfwGetKey('Q')) {
				if(x2==NULL) {
					X2IndustrialParams *p=new X2IndustrialParams;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p->loadFromFile("bestind.txt"))
							systemError("could not load x2 params");
					x2=new X2Industrial(p);
					followPart=x2->getCentralPart();
				}
			}
			if(glfwGetKey('T')) {
				if(crawler==NULL) {
					X2CrawlerParams *p=new X2CrawlerParams;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p->loadFromFile("bestind.txt"))
							systemError("could not load x2 crawler params");
					//crawler=new X2Crawler(p);
					crawler=new X2Crawler(p,true);
					followPart=crawler->getCentralPart();
				}
			}
			if(glfwGetKey('Y')) {
				if(crawler==NULL) {
					X2Industrial2Params *p=new X2Industrial2Params;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p->loadFromFile("bestind.txt"))
							systemError("could not load x2 industrial 2 params");
					industrial2=new X2Industrial2(p);
					followPart=industrial2->getCentralPart();
				}
			}
			if(glfwGetKey('U')) {
				if(quadro==NULL) {
					QuadroParams *p=new QuadroParams;
					if(glfwGetKey(GLFW_KEY_LCTRL)) 
						if(-1==p->loadFromFile("bestind.txt"))
							systemError("could not load quadroparams");
					quadro=new QuadroMachine(p);
					followPart=quadro->getCentralPart();
				}
			}

			/*if(camType==CAM_FOLLOW && x2) {
				setFollowTargetAndPanning(Vec3((float*)(x2->getPosition()+NxVec3(0,20,0)).get()),frameCount*0.0005f);
			}*/

			if(glfwGetKey(GLFW_KEY_F2)) setupTestObjectScene();
			if(glfwGetKey(GLFW_KEY_F3)) animateJoints=!animateJoints;
			lastTime=curTime;
		}

		updateTestObjects();
		if(animateJoints) animateAllJoints();
		if(!gFreeze)
			gScene->simulate(1.0f/60.0f);
		
		if(g_visualize) {
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
			updateGraphics();
			//glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
			//glClear( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT);
			if(g_enableGlow) {
				fbo->bind();
				//could activate some brightpass filter here
				updateGraphics(false,false);
				fbo->unBind();
				glow(fbo,0.75f);
				//add some noise and stuff later, then shadows and depth of field
			}
		}

		//må finne ut etterhvert hva som kan være hvor
		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		if(glfwGetKey( GLFW_KEY_LCTRL)) 
			renderDebugInfo();

		if(g_visualize)
			glfwSwapBuffers(); //screen update  - hvorfor kan man ikke kutte ut?
		else
			glfwPollEvents();

		if(imgRecorder)
			imgRecorder->update();

		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}

}

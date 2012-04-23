#include "physics.h"
#include "graphics/graphics.h"
#include "testobjects.h"
#include "graphics/MeshGraphicsObject.h"
#include "base/system.h"
#include "physx/MyCloth.h"
#include "graphics/glstuff.h"
#include "machines/machine.h"

//various machines for testing
#include "machines/legmachine.h"
#include "machines/x2industrial.h"
#include "machines/x2crawler.h"
#include "machines/quadromachine.h"
#include "hardware/quadrohardware.h"

//testing:
//#include "hardware/axservo.h"
//DynamixelDevice dxlDevice;
bool enableRealRobot=false;
QuadroHardware* quadroHW=NULL;


std::vector<Machine*> machines;
Part* followPart=NULL; //hack for follow cam
ImageRecorder* imgRecorder=NULL;
int frameCount=0; //hacky hack
float simTimeStep=1.0f/30.0f;

//experimental:
QuadroMachine* quadMachine=NULL;
	

//temp hack func
void cleanUpTestObjects()
{
	followPart=NULL;
	quadMachine=NULL;

	//temp
	if(quadroHW)
		delete quadroHW;
	quadroHW=NULL;

	for each(Machine* m in machines)
		delete m;
	machines.clear();
}

void updateTestObjects()
{
	if(!gFreeze) 
		frameCount++; //hacky hack
	
	if(followPart) { //camera following an object
		//setFollowTargetAndPanning(Vec3((float*)(followPart->act->getGlobalPosition()+NxVec3(0,20,0)).get()),frameCount*0.0002f);
		setFollowTargetAndPanning(Vec3((float*)(followPart->act->getGlobalPosition()+NxVec3(0,10,0)).get()),0);
	}

	//update all existing machines
	//also freeze this?? check
	for each(Machine* m in machines) {
		//m->update(frameCount/60.0f);
		m->update(frameCount*simTimeStep);
	}

	if(!gFreeze && enableRealRobot) {
		if(quadroHW)
			quadroHW->update(frameCount*simTimeStep);
		//if(quadroHW && frameCount%2==0) //only update robot every second frame (30 fps)
		//	quadroHW->update(frameCount*simTimeStep);
	}
}


void setupTestObjectScene()
{
	cleanUpTestObjects();
	terminatePhysics();
	initPhysics(); //hvor mye av dette må gjøres for reset??
	frameCount=0;
}

void addTestMachine(Machine *m)
{
	machines.push_back(m);
	followPart=m->getCentralPart();
}


void testObjectLoop()
{
	setupTestObjectScene();
	static double lastTime=0;
	bool animateJoints=false;
	bool running = true;
	while( running )
	{
		updateFPS();
		updateWindowSize();
		if(glfwGetKey(GLFW_KEY_TAB) && imgRecorder==NULL) {
			imgRecorder=new ImageRecorder(2,"c:/temp/video_out/testvideo",90);
			imgRecorder->start();
		}

		//selection of test objects
		Point3D pos(randRangef(-1,1)*32,40.0f, randRangef(-1,1)*32);
		Point3D dims(randRangef(0.5f,3.0f),randRangef(0.5f,3.0f),randRangef(0.5f,3.0f));
		if(glfwGetKey(GLFW_KEY_SPACE)) createBoxPart(pos,dims);
		if(glfwGetKey(GLFW_KEY_LSHIFT)) createCapsulePart(pos,dims.x*2,true,dims.y);
		double curTime=glfwGetTime();
		if(curTime-lastTime > 0.2) {
			int loadParams=glfwGetKey(GLFW_KEY_LCTRL);
			if(glfwGetKey('7'))  {
				LegParams p;
				if(loadParams && -1==p.loadFromFile("bestind.txt")) systemError("could not load leg params");
				addTestMachine(new LegMachine(p,LegMachine::CLOTH_MUSCLE));
			}
			if(glfwGetKey('Q')) {
				X2IndustrialParams *p=new X2IndustrialParams;
				if(loadParams && -1==p->loadFromFile("bestind.txt")) systemError("could not load x2 params");
				addTestMachine(new X2Industrial(p));
			}
			if(glfwGetKey('T')) {
				X2CrawlerParams *p=new X2CrawlerParams;
				if(loadParams && -1==p->loadFromFile("bestind.txt")) systemError("could not load x2 crawler params");
				addTestMachine(new X2Crawler(p,true));
			}
			if(glfwGetKey('U')) {
				QuadroParams *p=new QuadroParams;
				if(loadParams && -1==p->loadFromFile("bestind.txt")) systemError("could not load quadroparams");
				//addTestMachine(new QuadroMachine(p));
				QuadroMachine* m=new QuadroMachine(p);
				if(quadroHW) quadroHW->init(p);
				if(glfwGetKey(GLFW_KEY_RALT)) {
					char* playbackFileName="example_log_1.txt";
					m->loadPlaybackFile(playbackFileName);
					//m->loadPlaybackFile("fixed_positions.txt");
					if(quadroHW)
						quadroHW->loadPlaybackFile(playbackFileName);
				}
				addTestMachine(m);
				//experimental:
				//m->controlMode=QuadroMachine::MANUAL_CONTROL;
				quadMachine=m;
				
				gScene->setGravity(NxVec3(0,-9.81*100.0,0));
				simTimeStep=QUADRO_TIMESTEP;
				//m->enableControlLogging(true);
				//m->enableSimLogging(true);
				//if(quadroHW)
				//	quadroHW->enableHWLogging(true);
				
				//todo: sjekke vekt av center part etter å ha forandret density
				// - går det an å tilpasse større gravity?
				// - det hjelper om flere iterations - nå 50
				// - lese igjen om balanse i delers masse osv.
				// - prøve å tilpasse deler, se hva som ble gjort i muskeloppsett
				// - kanskje gjøre bein litt fyldigere
				// - må også gjøre dette sammen med testing av virkelig robot
				//todo: motor control implementation

			}
			if(glfwGetKey(GLFW_KEY_LCTRL) && glfwGetKey(GLFW_KEY_RALT)) {
				if(NULL==quadroHW)
					quadroHW=new QuadroHardware();
				enableRealRobot=true;
				simTimeStep=QUADRO_TIMESTEP;
				glfwSwapInterval(1); //setting time to "real" time


			}

			if(glfwGetKey(GLFW_KEY_F2)) setupTestObjectScene(); //this takes time, could consider cleaning up only objects
			if(glfwGetKey(GLFW_KEY_F3)) animateJoints=!animateJoints;

			//experimental manual joint control quadromachine
			/*if(quadMachine) {
				static float manualAngle=0;
				float step=0.2f;
				if(glfwGetKey('I')) manualAngle+=step;
				if(glfwGetKey('K')) manualAngle-=step;
				//manualAngle=clamp(manualAngle,QuadroParams::innerAngleMin,QuadroParams::innerAngleMax); //depends on joint
				unsigned curJoint=0;
				quadMachine->setJointTarget(curJoint,manualAngle);
				if(enableRealRobot) {
					int servoAngle=(int)angleToServo(manualAngle);
					printf("writing %d to servo\n",servoAngle);
					dxlDevice.setPosition(curJoint,servoAngle);
				}
			}*/

			lastTime=curTime;
		}

		if(animateJoints) 
			updateTestObjects();
		if(!gFreeze) {
			//have to read about the speed of simulation and check if more steps should be done before each visualization
			gScene->simulate(simTimeStep);
		}
		if(g_visualize)
			updateGraphics();

		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		if(glfwGetKey( GLFW_KEY_LCTRL)) 
			renderDebugInfo();
		if(g_visualize)
			glfwSwapBuffers(); //screen update
		else
			glfwPollEvents();
		if(imgRecorder)
			imgRecorder->update();

		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}
}


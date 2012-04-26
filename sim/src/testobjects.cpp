#include "physics.h"
#include "graphics/graphics.h"
#include "testobjects.h"
#include "graphics/MeshGraphicsObject.h"
#include "base/system.h"
#include "graphics/glstuff.h"
#include "machines/machine.h"

//various machines for testing
#include "machines/quadromachine.h"
#include "hardware/quadrohardware.h"

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
	double t0=glfwGetTime();
	cleanUpTestObjects();
	double t1=glfwGetTime();
	terminatePhysics();
	double t2=glfwGetTime();
	initPhysics(); //hvor mye av dette må gjøres 2or reset??
	double t3=glfwGetTime();
	printf("init times:\n cleanup internal data: %f\n terminate physics %f\n init physics %f\n total %f\n\n",t1-t0,t2-t1,t3-t2,t3-t0);
	frameCount=0;
}

void softReset()
{
	double t0=glfwGetTime();
	cleanUpTestObjects();
	//also: remove objects with physx calls
	//gPhysicsSDK->releaseScene(*gScene); ?? i så fall må scenen settes opp på nytt
	double t1=glfwGetTime();

	printf("soft reset: %f\n",t1-t0);

}

void addTestMachine(Machine *m)
{
	machines.push_back(m);
	followPart=m->getCentralPart();
}

void initRealRobot()
{
	quadroHW=new QuadroHardware();
	if(!quadroHW)
		systemError("could not init robot hardware");
	//should have some checking to see if the servos have been inited, since this is unstable
	enableRealRobot=true;
	glfwSwapInterval(1); //setting time to "real" time

}


void quadroLoop(const std::string &controlFileName, const std::string &logFileName, bool loop,bool useRealRobot)
{
	setupTestObjectScene();
	static double lastTime=0;
	bool animateJoints=false;
	bool running = true;

	if(!controlFileName.empty()) {
		QuadroParams *p=new QuadroParams;
		QuadroMachine* m=new QuadroMachine(p);
		gScene->setGravity(NxVec3(0,-9.81*100.0,0));
		simTimeStep=QUADRO_TIMESTEP;
		m->loadPlaybackFile(controlFileName.c_str(),loop);
		addTestMachine(m);
		quadMachine=m;
		m->enableControlLogging(true);
		if(!logFileName.empty())
			m->enableSimLogging(true,std::string(logFileName));
		else 
			m->enableSimLogging(true);
		animateJoints=true;

		if(useRealRobot) {
			initRealRobot();
			quadroHW->init(p);
			quadroHW->loadPlaybackFile(controlFileName.c_str(),loop);
			quadroHW->enableHWLogging(true);
		}
	}


	while( running )
	{
		updateFPS();
		updateWindowSize();
		if(glfwGetKey(GLFW_KEY_TAB) && imgRecorder==NULL) {
			imgRecorder=new ImageRecorder(2,"c:/temp/video_out/testvideo",90);
			imgRecorder->start();
		}

		//selection of test objects
		double curTime=glfwGetTime();
		if(curTime-lastTime > 0.2) {
			int loadParams=glfwGetKey(GLFW_KEY_LCTRL);
			if(glfwGetKey('U')) {
				QuadroParams *p=new QuadroParams;
				if(loadParams && -1==p->loadFromFile("log_bestind.txt")) systemError("could not load quadroparams");
				QuadroMachine* m=new QuadroMachine(p);
				gScene->setGravity(NxVec3(0,-9.81*100.0,0));
				simTimeStep=QUADRO_TIMESTEP;
				if(quadroHW) quadroHW->init(p);
				if(glfwGetKey(GLFW_KEY_RALT)) {
					char* playbackFileName="cmd_fixed_positions.txt";
					m->loadPlaybackFile(playbackFileName);
					if(quadroHW)
						quadroHW->loadPlaybackFile(playbackFileName);
				}
				addTestMachine(m);
				quadMachine=m;
				m->enableControlLogging(true);
				m->enableSimLogging(true);
				if(quadroHW)
					quadroHW->enableHWLogging(true);

			}
			if(glfwGetKey(GLFW_KEY_LCTRL) && glfwGetKey(GLFW_KEY_RALT)) {
				initRealRobot();
			}

			if(glfwGetKey(GLFW_KEY_F2)) setupTestObjectScene(); //this takes time, could consider cleaning up only objects
			if(glfwGetKey(GLFW_KEY_F3)) animateJoints=!animateJoints;

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

		//make some better logic for this later
		if(quadMachine && !controlFileName.empty() && quadMachine->playingBack==false)
			running=false;

	}
}


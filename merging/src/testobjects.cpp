#include "physics.h"
#include "graphics/graphics.h"
#include "testobjects.h"
#include "graphics/MeshGraphicsObject.h"
#include "base/system.h"
#include "graphics/glstuff.h"
#include "machines/machine.h"
#include <iostream>
#include <boost/foreach.hpp>
#include <math.h>
using namespace std;

//various machines for testing
#include "machines/quadromachine.h"
#include "hardware/quadrohardware.h"
#include "machines/quadrobot.h"

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

	BOOST_FOREACH(Machine* m, machines)
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
	BOOST_FOREACH(Machine* m, machines) {
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
	initPhysics(); //hvor mye av dette m� gj�res 2or reset??
	double t3=glfwGetTime();
	printf("init times:\n cleanup internal data: %f\n terminate physics %f\n init physics %f\n total %f\n\n",t1-t0,t2-t1,t3-t2,t3-t0);
	frameCount=0;
}

void softReset()
{
	double t0=glfwGetTime();
	cleanUpTestObjects();
	//also: remove objects with physx calls
	//gPhysicsSDK->releaseScene(*gScene); ?? i s� fall m� scenen settes opp p� nytt
	double t1=glfwGetTime();

	printf("soft reset: %f\n",t1-t0);

}

void addTestMachine(Machine *m)
{
	machines.push_back(m);
	followPart=m->getCentralPart();
}

void quadroLoop(const std::string &controlFileName, const std::string &logFileName, bool loop)
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
		//if(quadroHW) quadroHW->init(p);
		//if(quadroHW)
		//	quadroHW->loadPlaybackFile(playbackFileName);
		//if(quadroHW)
		//	quadroHW->enableHWLogging(true);
		m->loadPlaybackFile(controlFileName.c_str());
		addTestMachine(m);
		quadMachine=m;
		m->enableControlLogging(true);
		if(!logFileName.empty())
			m->enableSimLogging(true,std::string(logFileName));
		else 
			m->enableSimLogging(true);
		animateJoints=true;
	}

	int iter;
	iter = 0;
	//while( running )
	//for(ttt=1;ttt<=1;ttt++)
	//std::vector<PositionKey> m_positions; //storing prerecorded positions
	//int sizeVector;
	//sizeVector = m_positions.size();
	//cout << "m_positions = " << m_positions[sizeVector-1].t << endl;
	while (running)
	{
		iter++;
		cout << "testobjects quadroloop" << endl;
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
					char* playbackFileName="example_log_1.txt";
					//m->loadPlaybackFile(playbackFileName);
					m->loadPlaybackFile("cmd_fixed_positions.txt");
					if(quadroHW)
						quadroHW->loadPlaybackFile(playbackFileName);
				}
				addTestMachine(m);
				quadMachine=m;
				//m->enableControlLogging(true);
				//m->enableSimLogging(true);
				//if(quadroHW)
				//	quadroHW->enableHWLogging(true);

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
		//cout << "running = " << running << endl;
		//make some better logic for this later
		if(quadMachine && !controlFileName.empty() && quadMachine->playingBack==false)
			running=false;
		//cout << "running2 = " << running << endl;
		//cout << " playingBack =  " << quadMachine->playingBack << endl;
		//cout << "quadMachine = " << quadMachine << endl;
		//cout << "controlFileName.empty() = " << !controlFileName.empty() << endl;
		//if(ceil (m_positions[sizeVector-1].t*60 ) < iter )
			//running = false;
	}
}


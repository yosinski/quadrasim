#include "physics.h"
//#include "graphics/graphics.h"
#include "testobjects.h"
//#include "graphics/MeshGraphicsObject.h"
#include "base/system.h"
//#include "graphics/glstuff.h"
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
//ImageRecorder* imgRecorder=NULL;
int frameCount=0; //hacky hack
float simTimeStep=1.0f/30.0f;

//experimental:
QuadroMachine* quadMachine=NULL;
	

//temp hack func
void cleanUpTestObjects()
{
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
	
	//update all existing machines
	//also freeze this?? check
	BOOST_FOREACH(Machine* m, machines) {
		m->update(frameCount*simTimeStep);
	}

	if(!gFreeze && enableRealRobot) {
		if(quadroHW)
			quadroHW->update(frameCount*simTimeStep);
	}
}


void setupTestObjectScene()
{
	cleanUpTestObjects();
	terminatePhysics();
	initPhysics(); //hvor mye av dette m� gj�res 2or reset??
	frameCount=0;
}

void softReset()
{
	cleanUpTestObjects();
	//also: remove objects with physx calls
}

void addTestMachine(Machine *m)
{
	machines.push_back(m);
}

void quadroLoop(const std::string &controlFileName, const std::string &logFileName, bool loop)
{
	setupTestObjectScene();
	bool animateJoints=false;
	bool running = true;
	if(!controlFileName.empty()) {
	  cout << "i'm in !controlFileName.empty()" << endl;
		QuadroParams *p=new QuadroParams;
		cout << "done loading QuadroParams" << endl;
		QuadroMachine* m=new QuadroMachine(p);
		gScene->setGravity(NxVec3(0,-9.81*100.0,0));
		simTimeStep=QUADRO_TIMESTEP;
		cout << "simTimeStep = " << simTimeStep << endl;
		m->loadPlaybackFile(controlFileName.c_str());
		cout << "done loading playback file" << endl;
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
	while (running)
	{
		iter++;
		cout << "i'm in the while(running) loop! And iter++ =  " << iter  <<  endl;
		if(animateJoints) 
			updateTestObjects();
		if(!gFreeze) {
			gScene->simulate(simTimeStep);
		}

		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		if(quadMachine && !controlFileName.empty() && quadMachine->playingBack==false)
			running=false;
	}
}


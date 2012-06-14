//base Trepen robot class which both simulated and hardware versions can inherit
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "quadrobot.h"
#include "../base/tools.h"
#include <math.h>
#include <iostream>
#include <vector>
using namespace std;


//move to tools later
//add arbitrary src range
//maps val from [-1..+1] to [destMin,destMax]
float mapLinear(float val,float destMin,float destMax)
{
	return (val+1.0f)/2.0f*(destMax-destMin)+destMin;
}


float servoToAngle(float servoPos)
{
	//return float(servoPos/1023.0*2*PI-PI);
	return float((servoPos/1023.0*2-1)*150.0/180.0*PI); //servo ranges from -150 to 150 degrees
}

float angleToServo(float angle)
{
	//return float((angle+PI)/(2*PI)*1023.0);
	return float((angle/(PI*150.0/180.0)*0.5+0.5)*1023.0); //servo ranges from -150 to 150 degrees
}


void Quadrobot::loadPlaybackFile(const char* fileName, bool loopPlayback/* =true */)
{
	std::ifstream inFile(fileName,std::ios::in);
	if(!inFile.good()) 
	  exit(1);
	  cout << "could not open file" << endl;
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
	controlMode=PLAYBACK_CONTROL;
	m_loopPlayback=loopPlayback;
	playingBack=true;
	printf("machine in playback mode\n");
}


void Quadrobot::playbackControl(float simulationTime)
{
	//todo:
	//- compare plots

	if(m_positions.size()==0) {
		printf("no stored position keys\n");
		return;
	}

	//experimental:
	//float timeScale=0.5f;
	float timeScale=1.0f;
	simulationTime*=timeScale;
	//we could instead speed up the simulated motor, check which parameters are ok for this. p const, max speed or max force?

    if(m_loopPlayback)
        simulationTime=fmodf(simulationTime,m_positions.back().t);

    if(!m_loopPlayback && simulationTime > m_positions.back().t)
        playingBack=false;

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

	for(int i=0;i<NUM_JOINTS;i++) {
		float servoPos=res.values[i]; 
		float angularPos=servoToAngle(servoPos);
		setJointTarget(i,angularPos);
	}
}


void Quadrobot::setJointTarget(unsigned jointNumber,float angle)
{
	//the update function will control from these targets
	if(jointNumber<m_jointTargets.size())
		m_jointTargets[jointNumber]=angle;
}

float Quadrobot::getJointTarget(unsigned jointNumber)
{
	if(jointNumber<m_jointTargets.size())
		return m_jointTargets[jointNumber];
	else 
		return 0;
}


//-1 to +1
float Quadrobot::calcSinEnvelopeFromParams(float attackDuration,float p0Duration,float decayDuration,float p1Duration,float t,float phase)
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
	t=fmax(t-phase,0); //subtract phase but wait in the beginning so no abrupt movements
	t+=t1/2.0f; //want to start with muscles in neutral position

	float remainder=fmod(t,1.0f);
	return sineEnvelope(remainder,t0,t1,t2,t3)*2-1;
};

void Quadrobot::parameterControl(float simulationTime)
{
	int i=0;
	for(int i=0;i<NUM_JOINTS;i++) {
		//float freq=0.05f;
		float freq=0.5f; //think about where to control this, maybe only in simulated version?
		//float freq=1.0f; //think about where to control this, maybe only in simulated version?
		//float freq=1.5f; //think about where to control this, maybe only in simulated version?
		float phase=m_params->getValue("phase",i); //could also have accumulated phase.. more interesting for other machines?
		float attackTime=m_params->getValue("attack",i);
		float p0Time=m_params->getValue("p0",i);
		float decayTime=m_params->getValue("decay",i);
		float p1Time=m_params->getValue("p1",i);

		float envelope=calcSinEnvelopeFromParams(attackTime,p0Time,decayTime,p1Time,freq*simulationTime,phase);

		/*//map to parameter range
		float limLow=m_params->getValue("limLow",i);
		float limHigh=m_params->getValue("limHigh",i);
		if(limLow>limHigh) { //now evolution allows the parameters to overlap - swap if they do
			float temp=limHigh;
			limHigh=limLow;
			limLow=temp;
		}
		angularPos=mapLinear(angularPos,limLow,limHigh);*/

		float angleMin=m_jointLimitAngles[i].first;
		float angleMax=m_jointLimitAngles[i].second;
		float angularRange=angleMax-angleMin;
		float amplitude=m_params->getValue("amplitude",i);
		float center=m_params->getValue("centerAngle",i);
		float angularPos=center+envelope*amplitude*(angularRange/2.0f);
		//må finne riktig pos for center
		mapLinear(center,angleMin,angleMax); //nå er ikke center mellom 0..1 (-1 -> +1)

		//also check now if amplitude calculation is correct!

		setJointTarget(i,angularPos);
	}
}

void Quadrobot::updateControl(float time)
{
	if(controlMode==PLAYBACK_CONTROL)
		playbackControl(time);
	else if(controlMode==PARAMETERED_CONTROL)
		parameterControl(time);

	//maybe keep 2 separate files: one for controller stufvalues and one for simulator/hardware command and readback values (and positions)
	//but use the same logging enable signals in order to log at the same frames 
	//	fprintf(controllerOut,"#simtime (jointctrlval(normalized) jointctrlval actualjointangle)x%d xpos ypos absdeltapos\n",m_robJoints.size());

	//log controller time
	if(logControl && controlLogFile) fprintf(controlLogFile,"%.3f ",time); 
	

	//check joint limits and clip if necessary 

	for(unsigned i=0;i<m_jointTargets.size();i++) {
		float angularPos=m_jointTargets[i];
		if(logControl && controlLogFile) fprintf(controlLogFile,"%.3f ",angularPos); //log control angles
		if(logControl && controlLogFile) fprintf(controlLogFile,"%.1f ",angleToServo(angularPos)); //log control angles (servo coordinates)

		std::pair<float,float> lims;
		lims=m_jointLimitAngles[i];

		if(angularPos<lims.first) angularPos=lims.first; 
		if(angularPos>lims.second) angularPos=lims.second; 
		if(logControl && controlLogFile) fprintf(controlLogFile,"%.1f ",angleToServo(angularPos)); //log clipped control angles 

		setJointTarget(i,angularPos); //write clipped value back to joint targets
	}

	if(logControl && controlLogFile) fprintf(controlLogFile,"\n");

	//logging:
	//print interval
	//joint control value
	//clipped control value
	//actual read actuator value (sim / hw)
	//read position values (sim / hw)

	//static float nextIntervalPrint=0;
	//bool debugPrint=false;
	//	if(simulationTime>nextIntervalPrint) {
	//		debugPrint=true;
	//		nextIntervalPrint+=PRINT_INTERVAL;
	//	}
	//}
}

void Quadrobot::init(QuadroParams* params /* =NULL */ )
{
	if(NULL==params)
		params=new QuadroParams();
	assert(params);
	//params->print();
	m_params=params;
	controlMode=PARAMETERED_CONTROL;
	m_jointTargets.resize(NUM_JOINTS,0);

	//absolute security limits for the joints
	//these are specified in servo coordinates, [0..1023] maps to [-150..150] degrees (storing radians);
	m_innerAngleMin=servoToAngle(150); //-362
	m_innerAngleMax=servoToAngle(770); //+258
	m_outerAngleMin=servoToAngle(30);
	m_outerAngleMax=servoToAngle(940);
	m_coreAngleMin=servoToAngle(512-180);
	m_coreAngleMax=servoToAngle(512+180);


	typedef std::pair<float,float> LimPair;
	for(int i=0;i<4;i++) {
		m_jointLimitAngles.push_back(LimPair(m_innerAngleMin,m_innerAngleMax)); //angular limits inner joint
		m_jointLimitAngles.push_back(LimPair(m_outerAngleMin,m_outerAngleMax)); //angular limits outer joint
	}
	m_jointLimitAngles.push_back(LimPair(m_coreAngleMin,m_coreAngleMax)); //angular limits core

	controlLogFile=NULL;
	logControl=false;
}


void Quadrobot::enableControlLogging(bool enable)
{
	//later: add option for logging on specific frames - auto-swith off ?
	std::string logFileName="log_controller.txt";
	logControl=enable;
	if(controlLogFile==NULL) {
		printf("opening log file %s for writing\n",logFileName.c_str());
		if(!(controlLogFile=fopen(logFileName.c_str(),"w"))) {
			printf("could not open log file %s for writing\n",logFileName.c_str());
			logControl=false;
		}
	}
	fprintf(controlLogFile,"# controltime (jointctrlval jointctrlval_servocoord jointctrlval_servocoord_clipped)x%d\n",NUM_JOINTS);
}

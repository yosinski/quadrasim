#ifndef _QUADROBOT_H
#define _QUADROBOT_H


#include "../base/floatparammultivallist.h"
#include <vector>

#define NUM_JOINTS 9

float servoToAngle(float servoPos); //servoPos: 0-1023  return angle: -PI*150/180, PI*150/180  (servo range: +/- 150 degrees)
float angleToServo(float angle); //angle: -PI, PI  return servo position: 0-1023 (if angles are in servo range)


struct PositionKey 
{
	float t;
	float values[NUM_JOINTS];
};



//////////////////////////////////////////////////////////////////////////

struct QuadroParams : public FloatParamMultiValList
{
	//these are constant parameters, not to be evolved
	int numJoints;

	QuadroParams(){
		numJoints=NUM_JOINTS;
		//n�: limlow/high er -1..1, blir oversatt til -PI..PI
		//can introduce fewer parameters later by exploiting symmetry
		//NOTE! now there is no checking of combinations of joint angles resulting in invalid/HAZARDOUS configurations (e.g. both inner and outer joints are min)


		//now: setting joint movement as -1 to +1 range for init angle and 0..1 for amplitude
		addParam("centerAngle",FloatParam(0.5f,-0.5f,0.2f,6)); //inner
		addParam("amplitude",FloatParam(0.1f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(-0.1f,-0.5f,0.3f,6)); //outer
		addParam("amplitude",FloatParam(0.3f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(0.5f,-0.5f,0.2f,6)); //inner
		addParam("amplitude",FloatParam(0.1f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(-0.1f,-0.5f,0.3f,6)); //outer
		addParam("amplitude",FloatParam(0.3f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(0.5f,-0.5f,0.2f,6)); //inner
		addParam("amplitude",FloatParam(0.1f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(-0.1f,-0.5f,0.3f,6)); //outer
		addParam("amplitude",FloatParam(0.3f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(0.5f,-0.5f,0.2f,6)); //inner
		addParam("amplitude",FloatParam(0.1f,0.05f,0.25f,6));
		addParam("centerAngle",FloatParam(-0.1f,-0.5f,0.3f,6)); //outer
		addParam("amplitude",FloatParam(0.3f,0.05f,0.25f,6));
		//center joint
		addParam("centerAngle",FloatParam(0.0f,-0.2f,0.2f));
		addParam("amplitude",FloatParam(0.0f,0.0f,0.3f,6));

		/*//now: setting joint movement as -1 to +1 range for init angle and 0..1 for amplitude
		setParams("centerAngle",FloatParam(-0.1f,-0.5f,0.3f,6),numJoints-1);
		setParams("amplitude",FloatParam(0.2f,0.05f,0.25f,6),numJoints-1);
		//center joint
		addParam("centerAngle",FloatParam(0.0f,-0.2f,0.2f));
		addParam("amplitude",FloatParam(0.2f,0.0f,0.3f,6));
		*/


		//leg joints
		//now setting some crazy phases
		addParam("phase",FloatParam(0.0f,0,1,6));
		addParam("phase",FloatParam(0.1f,0,1,6));
		addParam("phase",FloatParam(0.2f,0,1,6));
		addParam("phase",FloatParam(0.3f,0,1,6));
		addParam("phase",FloatParam(0.4f,0,1,6));
		addParam("phase",FloatParam(0.5f,0,1,6));
		addParam("phase",FloatParam(0.6f,0,1,6));
		addParam("phase",FloatParam(0.7f,0,1,6));
		addParam("phase",FloatParam(0.8f,0,1,6));


		//check how phase works - does it make sense to limit from 0 to 1?
		//setParams("phase",FloatParam(0.3f,0,1),numJoints-1);
		setParams("attack",FloatParam(0.5f,0.3f,0.7f,4),numJoints-1); 
		setParams("p0",FloatParam(0.1f,0,0.5f,4),numJoints-1); 
		setParams("decay",FloatParam(0.5f,0.3f,0.7f,4),numJoints-1);
		setParams("p1",FloatParam(0.2f,0,0.5f,4),numJoints-1);

		//central joint
		addParam("phase",FloatParam(0.5f,0,1,6));
		addParam("attack",FloatParam(0.5f,0.3f,0.7f,4)); 
		addParam("p0",FloatParam(0.1f,0,0.3f,4)); 
		addParam("decay",FloatParam(0.5f,0.3f,0.7f,4));
		addParam("p1",FloatParam(0.2f,0,0.3f,4));

	}
};

//////////////////////////////////////////////////////////////////////////



class Quadrobot
{
public:
	enum ControlMode { IDLE, PARAMETERED_CONTROL, MANUAL_CONTROL, PLAYBACK_CONTROL };
	
	//M� MAN HA CONSTRUCTOR HER?
	void init(QuadroParams* params =NULL );

	//QuadroMachine(QuadroParams* params=NULL);
	//void loadPlaybackFile(const char* fileName, bool loopPlayback=true);
	void loadPlaybackFile(const char* fileName);
	void setJointTarget(unsigned jointNumber,float angle);
	float getJointTarget(unsigned jointNumber);
	int controlMode;
	bool playingBack;

	void updateControl(float time);
	
	void enableControlLogging(bool enable);
	//void setLogInterval(float time);

protected:
	void parameterControl(float simulationTime);
	void playbackControl(float simulationTime);
	//void controlManual(float simulationTime);
	float calcSinEnvelopeFromParams(float attackTime,float p0Time,float decayTime,float p1Time,float t,float phase); //calculates position in a "sine envelope function" from pause parameter values (period is normalized)

	std::vector<float> m_jointTargets;
	std::vector<std::pair<float,float> > m_jointLimitAngles;
	float m_innerAngleMin;
	float m_innerAngleMax;
	float m_outerAngleMin;
	float m_outerAngleMax;
	float m_coreAngleMin;
	float m_coreAngleMax;

	std::vector<PositionKey> m_positions; //storing prerecorded positions
	bool m_loopPlayback;

	QuadroParams *m_params;

	FILE* controlLogFile;
	bool logControl;
};

#endif

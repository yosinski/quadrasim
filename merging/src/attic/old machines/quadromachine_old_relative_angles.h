#ifndef QUADROMACHINE_H
#define QUADROMACHINE_H

//base class for linear actuator configurations
#include "machine.h"
#include "../base/floatparammultivallist.h"
#include "../part.h"
#include "../graphics/MeshGraphicsObject.h"
#include <vector>
#include "../evolutionharness.h"

#define NUM_JOINTS 9


struct PositionKey 
{
	float t;
	float values[NUM_JOINTS];
};

//////////////////////////////////////////////////////////////////////////

struct QuadroParams : public FloatParamMultiValList
{
	QuadroParams::QuadroParams() {
		//deduce initAngle and limitAmp from limLow and limHigh, easier for control system
		//but keep limlow/high as parameters because easiest to specify joint min/max
		//can also change to one initial angle + amplitude
		//nå: limlow/high er -1..1, blir oversatt til -PI..PI

		//can introduce fewer parameters later by exploiting symmetry

		//leg joints
		int numJoints=8; 
		setParams("limLow",FloatParam(1.0f,0.01f,0.3f,6),numJoints); 
		setParams("limHigh",FloatParam(1.0f,0.01f,0.3f,6),numJoints);
		//central joint - #8
		addParam("limLow",FloatParam(1.0f,0.01f,0.3f,6));
		addParam("limHigh",FloatParam(1.0f,0.01f,0.3f,6));
		

		//leg joints
		setParams("phase",FloatParam(0.5f,0,1),numJoints);
		setParams("attack",FloatParam(0.5f,0.2f,0.7f,6),numJoints); 
		setParams("p0",FloatParam(0.1f,0,0.6f,6),numJoints); 
		setParams("decay",FloatParam(0.5f,0.2f,0.7f,6),numJoints);
		setParams("p1",FloatParam(0.2f,0,0.6f,6),numJoints);

		//central joint
		addParam("phase",FloatParam(0.5f,0,1));
		addParam("attack",FloatParam(0.5f,0.3f,0.7f,6)); 
		addParam("p0",FloatParam(0.1f,0,0.4f,6)); 
		addParam("decay",FloatParam(0.5f,0.3f,0.7f,6));
		addParam("p1",FloatParam(0.2f,0,0.4f,6));

	}
};

//////////////////////////////////////////////////////////////////////////


class QuadroMachine : public Machine
{
public:
	QuadroMachine(QuadroParams* params=NULL);
	void update(float simulationTime);
	void loadPlaybackFile(const char* fileName);
	NxVec3 getPosition(void);
	Part* getCentralPart(void);

protected:
	void control(float simulationTime);
	void playback(float simulationTime);

	float servoToAngle(float servoPos); //servoPos: 0-1023  return angle: -PI, PI
	float angleToServo(float angle); //angle: -PI, PI  return servo position: 0-1023
	
	//this one could maybe be common for machine?
	Part* createMachinePart(NxActorDesc& actorDesc, NxActor* attachTo, NxMat34& transform, bool glue);
	//this one could maybe be common for machine?
	float calcSinEnvelopeFromParams(float attackTime,float p0Time,float decayTime,float p1Time,float t,float phase); //calculates position in a "sine envelope function" from pause parameter values (period is normalized)

	NxRevoluteJoint* createParamMotorJoint(NxActor* p1,NxVec3 p1LocalAxis,NxActor* p2,int jointParamNumber);
	void controlRevoluteMotorP(NxRevoluteJoint* joint, float targetAngle,float maxSpeed,float proportionalFactor);
	//consider moving controlmotor also to machine?
	//or create motormachine

	Part* createPCCore(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createBatteryCore(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createUpperLeg(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createLowerLeg(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createLeg(NxActor* attachTo, NxMat34& transform, bool glue,int jointNumber);
	void initMeshes();
	float getInitAngle(int jointNumber);
	
	static MeshGraphicsObject* m_pcPartMesh;
	static MeshGraphicsObject* m_batteryPartMesh;
	static MeshGraphicsObject* m_upperLegMesh;
	static MeshGraphicsObject* m_lowerLegMesh;

	std::vector<Part*> m_robParts;
	std::vector<NxRevoluteJoint*> m_robJoints; //only the revolute motors

	std::vector<PositionKey> m_positions; //storing prerecorded positions
	bool playbackMode;

	//could this also be moved down to machine class?
	FloatParamMultiValList *m_params; //used by all subclasses   
};




///////////////////////////////////////////////////////////

class QuadroEvolutionHarness : public EvolutionHarness
{
public:
	QuadroEvolutionHarness(int seed);
};




#endif
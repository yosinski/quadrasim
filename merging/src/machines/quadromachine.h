#ifndef QUADROMACHINE_H
#define QUADROMACHINE_H

//sjekk kode med p-kontroll
//vurdere 6DOF
//rydde opp mer i testobjects

//kan legge inn balanse i genome med amplitude vs. kurvehastighet. begge deler ikke mulig


#include "machine.h"
#include "quadrobot.h"
#include "../part.h"
#include "../graphics/MeshGraphicsObject.h"
#include <vector>
#include "../evolutionharness.h"

#define QUADRO_TIMESTEP (1/60.0f)
//#define QUADRO_TIMESTEP (1/30.0f)

class QuadroMachine : public Machine, public Quadrobot
{
public:
	QuadroMachine(QuadroParams* params=NULL);
	void update(float simulationTime);
	NxVec3 getPosition(void);
	Part* getCentralPart(void);
	void enableSimLogging(bool enable);


protected:
	Part* createMachinePart(NxActorDesc& actorDesc, NxActor* attachTo, NxMat34& transform, bool glue);
	NxRevoluteJoint* createMotorJoint(NxActor* p1,NxVec3 p1LocalAxis,NxActor* p2,float minAngle,float maxAngle);
	void controlRevoluteMotorP(NxRevoluteJoint* joint, float targetAngle,float maxSpeed,float proportionalFactor);

	Part* createPCCore(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createBatteryCore(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createUpperLeg(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createLowerLeg(NxActor* attachTo, NxMat34& transform, bool glue);
	Part* createLeg(NxActor* attachTo, NxMat34& transform, bool glue);
	void initMeshes();
	
	static MeshGraphicsObject* m_pcPartMesh;
	static MeshGraphicsObject* m_batteryPartMesh;
	static MeshGraphicsObject* m_upperLegMesh;
	static MeshGraphicsObject* m_lowerLegMesh;

	std::vector<Part*> m_robParts;
	std::vector<NxRevoluteJoint*> m_robJoints; //only the revolute motors

	FILE* simLogFile;
	bool logSim;
};




///////////////////////////////////////////////////////////

class QuadroEvolutionHarness : public EvolutionHarness
{
public:
	QuadroEvolutionHarness(int seed);
};




#endif

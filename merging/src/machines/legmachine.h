#ifndef LEGMACHINE_H
#define LEGMACHINE_H

#include "machine.h"
#include "../base/floatparamlist.h"
#include "../physx/MyCloth.h"
#include "../physics.h"
#include <map>
#include <string>
#include "../evolutionharness.h"
#include "muscle.h"


struct LegParams : public FloatParamList
{
	LegParams::LegParams() { //this is the only unique part

		paramList["platLen"]=FloatParam(6,4,8,5);
		paramList["topLegLen"]=FloatParam(5,3,6,6);
		paramList["botLegLen"]=FloatParam(4.5f,3,6,6);

		paramList["topLegRot"]=FloatParam(-0.36f,-0.7f,-0.1f,6);
		//paramList["botLegRot"]=FloatParam(0.7f,0.4f,1.0f,6);
		paramList["botLegRot"]=FloatParam(0.9f,0.6f,1.0f,6);
		paramList["botLegOffset"]=FloatParam(0.25f,0.0f,0.5f,6);
		
		//paramList["muscleAttachPointTopLegBack"]=FloatParam(-0.2f,-0.8f,0.8f,6);
		//paramList["muscleAttachPointTopLegFront"]=FloatParam(-0.3f,-0.8f,0.8f,6);
		//paramList["muscleAttachPointBotLegFront"]=FloatParam(0.5f,0.2f,1.0f,6);

		paramList["topMuscleAttachPointTopLegBack"] =FloatParam(0.1f,0.0f,0.75f,4); //low precision
		paramList["topMuscleAttachPointTopLegFront"]=FloatParam(0.1f,0.0f,0.75f,4);
		paramList["botMuscleAttachPointTopLegBack"] =FloatParam(0.1f,0.0f,0.75f,4);
		paramList["botMuscleAttachPointTopLegFront"]=FloatParam(0.3f,0.0f,0.75f,4);
		paramList["muscleAttachPointBotLegFront"]=FloatParam(0.7f,0.2f,1.0f,6);

		//control - general
		paramList["freq"]=FloatParam(0.5f,0.48f,0.52f,6);   //høy hastighet blir ustabilt?
		//paramList["freq"]=FloatParam(0.1f,0.08f,0.12f,6);   //høy hastighet blir ustabilt?
		paramList["phaseOtherLeg"]=FloatParam(0.5f,0.45f,0.55f,6);    //fase mellom venstre og høyre side
		//control for upper joint
		paramList["attackTop"]=FloatParam(0.49f,0.2f,0.6f,6); //attack time in envelope
		paramList["pauseTop"]=FloatParam(0.01f,0.0f,0.3f,6); //how long to stay in '1' 
		paramList["decayTop"]=FloatParam(0.49f,0.2f,0.7f,6); //decay time in envelope
		//control for lower joint
		paramList["phaseBelowKnee"]=FloatParam(0.1f,0,0.3f,6);    //fase mellom oppe og nede
		paramList["attackBot"]=FloatParam(0.15f,0.1f,0.4f,6); //attack time in envelope
		paramList["pauseBot"]=FloatParam(0.25f,0.1f,0.6f,6); //how long to stay in '1' 
		paramList["decayBot"]=FloatParam(0.15f,0.1f,0.4f,6); //decay time in envelope

	}

};


class LegMachine : public Machine
{
public:
	enum MuscleType{DISTANCE_MUSCLE,SPRING_MUSCLE,CLOTH_MUSCLE};

	LegMachine(LegParams& params,int muscleType=CLOTH_MUSCLE);
	void update(float simulationTime);
	Part* getCentralPart(void); 

	Muscle* muscles[8];
	Part* legParts[4];
	Part* topPart;
	bool touchSensors[2];

private:
	float createSingleLeg(Part* attachTo,Part** legParts,NxVec3 localAttachPoint,LegParams& params);
	void attachMuscles(Part* platform,Part** legPointers,Muscle** musclePointers,NxVec3 localAttachPoint,LegParams& params,int muscleType);
	void attachTouchSensor(Part* leg, bool* touchSensor, LegParams& params);

	float m_legWidth;
	LegParams m_params;
};


class LegEvolutionHarness : public EvolutionHarness
{
public:
	LegEvolutionHarness(int seed);
	~LegEvolutionHarness();
};


#endif

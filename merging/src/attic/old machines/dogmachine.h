#ifndef DOGMACHINE_H
#define DOGMACHINE_H

#include "../base/floatparamlist.h"
#include "../physx/MyCloth.h"
#include "../physics.h"
#include <map>
#include <string>
#include "../evolutionharness.h"

struct DogParams : public FloatParamList
{
	DogParams::DogParams() { //this is the only unique part

		//for forward walking - only morphologically symmetric left-right, not front-back
		//burde man ha benmodeller i renderingen? Burde vel fungere, modellere som capsule

		//could add desired bit precision for each floatparam?

		//some params
		//paramList["legtype"]=FloatParam(0,0,1);

		//platform length
		paramList["platlen"]=FloatParam(20,10,30,6);
		//paramList["platlen"]=FloatParam(6,20,40,6);

		//leg lengths front - back
		//paramList["leg1len"]=FloatParam(4,3,6,6);
		paramList["leg1len"]=FloatParam(3,1,4,5);
		//paramList["leg2len"]=FloatParam(4,3,6);

		//leg from edge - how much the leg is "indented" from the edge
		//paramList["legfromedge"]=FloatParam(0.8f,0.4f,0.85f,6);
		paramList["legfromedge"]=FloatParam(0.7f,0.6f,0.9f,5);

		//various muscle params
		paramList["muscleWidth"]=FloatParam(1.3f,1.0f,1.6f,5);  //evt ha 4 av disse, en for hver muskel


		paramList["leg1attlofront"]=FloatParam(0.8f,0.8f,1.0f,3); 
		paramList["leg1attloback"]=FloatParam(0.8f,0.8f,1.0f,3); 
		paramList["leg1atthiback"]=FloatParam(0.8f,0.1f,0.9f,5); //fra midten til starten av baksiden av beinet.. :-S

		paramList["leg2attlofront"]=FloatParam(0.8f,0.8f,1.0f,3);
		paramList["leg2attloback"]=FloatParam(0.8f,0.8f,1.0f,3);
		paramList["leg2atthifront"]=FloatParam(0.8f,0.1f,0.9f,5);


		//control!
		//må kontrollere 4 muskler på en side pluss faseforskyvning til den andre siden (1 param, ellers samme delays etc)


		//    ___
		//   /   \___
		//     p0  p1
		// up and down speed dependent on muscle vol
		//p1=1-p0
		paramList["pause0"]=FloatParam(0.2f,0.1f,0.6f,8); //forhold av/på for muskel0
		paramList["attack"]=FloatParam(0.05f,0.03f,0.1f,6); //attack time in envelope
		paramList["decay"]=FloatParam(0.6f,0.03f,0.8f,6); //decay time in envelope
		paramList["freq"]=FloatParam(0.8f,0.1f,1.0f,10);   //høy hastighet blir ustabilt?
		paramList["phase0"]=FloatParam(0.25f,0,1,8);    //fase mellom for- og bakben
		paramList["phase1"]=FloatParam(0.5f,0,1,8);    //fase mellom venstre og høyre side



		//successful "crawl"
		//(using the default values)
		//paramList["platlen"]=FloatParam(20,10,30,6);
		//paramList["leg1len"]=FloatParam(3,1,4,5);
		//paramList["legfromedge"]=FloatParam(0.7f,0.6f,0.9f,5);
		//paramList["muscleWidth"]=FloatParam(1.3f,1.0f,1.6f,5);  //evt ha 4 av disse, en for hver muskel
		//paramList["leg1attlofront"]=FloatParam(0.8f,0.8f,1.0f,3); 
		//paramList["leg1attloback"]=FloatParam(0.8f,0.8f,1.0f,3); 
		//paramList["leg1atthiback"]=FloatParam(0.8f,0.1f,0.9f,5); //fra midten til starten av baksiden av beinet.. :-S
		//paramList["leg2attlofront"]=FloatParam(0.8f,0.8f,1.0f,3);
		//paramList["leg2attloback"]=FloatParam(0.8f,0.8f,1.0f,3);
		//paramList["leg2atthifront"]=FloatParam(0.8f,0.1f,0.9f,5);
		//paramList["pause0"]=FloatParam(0.2f,0.1f,0.6f,8); //forhold av/på for muskel0
		//paramList["attack"]=FloatParam(0.05f,0.03f,0.2f,6); //attack time in envelope
		//paramList["decay"]=FloatParam(0.6f,0.05f,0.8f,6); //decay time in envelope
		//paramList["freq"]=FloatParam(0.6f,0.1f,1.0f,10);   //høy hastighet blir ustabilt?
		//paramList["phase0"]=FloatParam(0.25f,0,1,8);    //fase mellom for- og bakben
		//paramList["phase1"]=FloatParam(0.5f,0,1,8);    //fase mellom venstre og høyre side

		//successful "trot"
		//morphology values same as above
		//paramList["pause0"]=FloatParam(0.4f,0.1f,0.6f,8); //forhold av/på for muskel0
		//paramList["attack"]=FloatParam(0.05f,0.03f,0.1f,6); //attack time in envelope
		//paramList["decay"]=FloatParam(0.5f,0.03f,0.8f,6); //decay time in envelope
		//paramList["freq"]=FloatParam(1.0f,0.1f,2.0f,10);   //høy hastighet blir ustabilt?
		//paramList["phase0"]=FloatParam(0.5f,0,1,8);    //fase mellom for- og bakben
		//paramList["phase1"]=FloatParam(0.5f,0,1,8);    //fase mellom venstre og høyre side

		//successful "pace"
		//paramList["pause0"]=FloatParam(0.4f,0.1f,0.6f,8); //forhold av/på for muskel0
		//paramList["attack"]=FloatParam(0.05f,0.03f,0.1f,6); //attack time in envelope
		//paramList["decay"]=FloatParam(0.5f,0.03f,0.8f,6); //decay time in envelope
		//paramList["freq"]=FloatParam(1.0f,0.1f,2.0f,10);   //høy hastighet blir ustabilt?
		//paramList["phase0"]=FloatParam(0.0f,0,1,8);    //fase mellom for- og bakben
		//paramList["phase1"]=FloatParam(0.5f,0,1,8);    //fase mellom venstre og høyre side

	}

};


//make this inherit "machine" later
class DogMachine
{
public:
	DogMachine(DogParams& params);
	void update(float simulationTime);

	MyCloth* muscles[8];
	Part* topPart;
	bool touchSensors[4];

private:
	MyCloth* createMuscle(NxVec3& startPos,NxVec3& endPos,float radius,int heightDivs,int aroundDivs);

	float m_p0;
	float m_attack;
	float m_decay;
	float m_frequency;
	float m_phase0;
	float m_phase1;

	FILE *debugFile;

};


class DogEvolutionHarness : public EvolutionHarness
{
public:
	DogEvolutionHarness(int seed);
	~DogEvolutionHarness();

};




#endif

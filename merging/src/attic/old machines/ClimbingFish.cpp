#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
using namespace std;

#include "ClimbingFish.h"
#include "../PhysX/DynamicActor.h"
#include "../PhysX/joints.h"
#include "../physics.h"
#include "../graphics/graphics.h"

struct Body : public DynamicActor {
	Body(NxScene& scene, const NxVec3& position, const NxMat33& orientation = NxMat33(NX_IDENTITY_MATRIX)) : DynamicActor(scene, position, orientation) {
		float sideLength = 2;
		NxVec3 ne(sideLength / 2, sideLength / 2, 0);
		NxVec3 nw(-sideLength / 2, sideLength / 2, 0);
		NxVec3 sw(-sideLength / 2, -sideLength / 2, 0);
		NxVec3 se(sideLength / 2, -sideLength / 2, 0);
		NxVec3 bottom(0, 0, -1);

		limbAttachmentPoints.resize(4);
		limbAttachmentPoints[0].push_back(bottom);
		limbAttachmentPoints[0].push_back(ne);
		limbAttachmentPoints[0].push_back(se);
		limbAttachmentPoints[1].push_back(bottom);
		limbAttachmentPoints[1].push_back(nw);
		limbAttachmentPoints[1].push_back(ne);
		limbAttachmentPoints[2].push_back(bottom);
		limbAttachmentPoints[2].push_back(sw);
		limbAttachmentPoints[2].push_back(nw);
		limbAttachmentPoints[3].push_back(bottom);
		limbAttachmentPoints[3].push_back(se);
		limbAttachmentPoints[3].push_back(sw);

		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 3; ++j)
				addCapsule(limbAttachmentPoints[i][j], limbAttachmentPoints[i][(j + 1) % 3], .1);
	}

	vector<NxVec3> getLimbAttachmentPoints(int iLimb) {
		vector<NxVec3> points;
		for(int i = 0; i < 3; ++i)
			points.push_back(transformToZUpGlobal(limbAttachmentPoints[iLimb][i]));
		return points;
	}

	vector<vector<NxVec3>> limbAttachmentPoints;
};

struct TripartiteLimb {
	TripartiteLimb(NxScene& scene, Body& body, int iLimb, ClimbingFishParams& params) {
		vector<NxVec3> p = body.getLimbAttachmentPoints(iLimb);

		NxVec3 center = (p[0] + p[1] + p[2]) / 3;
		NxVec3 normal = (p[1] - p[0]).cross(p[2] - p[1]);  normal.normalize();
		normal *= ((p[1] - p[0]).magnitude() + (p[2] - p[1]).magnitude() + (p[0] - p[2]).magnitude()) / 3;
		NxVec3 footPosition = center + normal;

		thigh1 = new Thigh(scene, p[0], footPosition, center);
		thigh2 = new Thigh(scene, p[1], footPosition, center);
		thigh3 = new Thigh(scene, p[2], footPosition, center);
		calf1 = new Calf(scene, p[0], footPosition, center);
		calf2 = new Calf(scene, p[1], footPosition, center);
		calf3 = new Calf(scene, p[2], footPosition, center);
		foot = new Foot(scene, footPosition, center);

		addIsoUniversalJoint(scene, body, *thigh1, p[0], thigh1->getXAxis());
		addIsoUniversalJoint(scene, body, *thigh2, p[1], thigh2->getXAxis());
		addIsoUniversalJoint(scene, body, *thigh3, p[2], thigh3->getXAxis());

		actuator1 = new CylindricalJoint(scene, *thigh1, *calf1, thigh1->getEnd(), thigh1->getXAxis());
		actuator2 = new CylindricalJoint(scene, *thigh2, *calf2, thigh2->getEnd(), thigh2->getXAxis());
		actuator3 = new CylindricalJoint(scene, *thigh3, *calf3, thigh3->getEnd(), thigh3->getXAxis());

		addRevoluteJoint(scene, *calf1, *foot, footPosition, calf1->getZAxis());
		addRevoluteJoint(scene, *calf2, *foot, footPosition, calf2->getZAxis());
		addRevoluteJoint(scene, *calf3, *foot, footPosition, calf3->getZAxis());

		controller1.attackPhase = params.getValue("actuatorAttackPhase", 3*iLimb);
		controller1.releasePhase = params.getValue("actuatorReleasePhase", 3*iLimb);
		controller2.attackPhase = params.getValue("actuatorAttackPhase", 3*iLimb + 1);
		controller2.releasePhase = params.getValue("actuatorReleasePhase", 3*iLimb + 1);
		controller3.attackPhase = params.getValue("actuatorAttackPhase", 3*iLimb + 2);
		controller3.releasePhase = params.getValue("actuatorReleasePhase", 3*iLimb + 2);
	}

	~TripartiteLimb() {
		delete thigh1;
		delete thigh2;
		delete thigh3;
		delete calf1;
		delete calf2;
		delete calf3;
		delete foot;
		delete actuator1;
		delete actuator2;
		delete actuator3;
	}

	CylindricalJoint& getActuator1() { return *actuator1; }
	CylindricalJoint& getActuator2() { return *actuator2; }
	CylindricalJoint& getActuator3() { return *actuator3; }

	void update(float simulationTime) {
		actuator1->setTargetDistance(controller1.getPosition(simulationTime));
		actuator2->setTargetDistance(controller2.getPosition(simulationTime));
		actuator3->setTargetDistance(controller3.getPosition(simulationTime));

		actuator1->update(simulationTime);
		actuator2->update(simulationTime);
		actuator3->update(simulationTime);
	}

	struct Thigh : public DynamicActor {
		Thigh(NxScene& scene, const NxVec3& bodyAttachmentPoint, const NxVec3& footPosition, const NxVec3& limbCenter)
			: DynamicActor(scene, bodyAttachmentPoint, makeOrientation(footPosition - bodyAttachmentPoint, limbCenter - bodyAttachmentPoint)) {
			float d = (footPosition - bodyAttachmentPoint).magnitude();
			start = NxVec3(.3, 0, 0);
			end = NxVec3(d / 2, 0, 0);
			addCapsule(start, end, .1);
		}

		NxVec3 getStart() { return transformToZUpGlobal(start); }
		NxVec3 getEnd() { return transformToZUpGlobal(end); }

		NxVec3 start;
		NxVec3 end;
	};

	struct Calf : public DynamicActor {
		Calf(NxScene& scene, const NxVec3& bodyAttachmentPoint, const NxVec3& footPosition, const NxVec3& limbCenter)
			: DynamicActor(scene, bodyAttachmentPoint, makeOrientation(footPosition - bodyAttachmentPoint, limbCenter - bodyAttachmentPoint)) {
			float d = (footPosition - bodyAttachmentPoint).magnitude();
			addCapsule(NxVec3(d / 2 + .3, 0, 0), NxVec3(d - .3, 0, 0), .05);
		}
	};

	struct Foot : public DynamicActor {
		Foot(NxScene& scene, const NxVec3& position, const NxVec3& limbCenter)
			: DynamicActor(scene, position, makeOrientation(position - limbCenter)) {
			addCapsule(NxVec3(0, 0, 0), NxVec3(.01, 0, 0), .3);
		}
	};

	struct JointController {
		JointController() : frequency(.1), attackPhase(0), releasePhase(.5) {}

		float getPosition(float simulationTime) {
			float phase = fmodf(simulationTime * frequency - attackPhase, 1);
			return phase < releasePhase;
		}

		float frequency;
		float attackPhase;
		float releasePhase;
	};

	Thigh* thigh1;
	Thigh* thigh2;
	Thigh* thigh3;
	Calf* calf1;
	Calf* calf2;
	Calf* calf3;
	Foot* foot;

	CylindricalJoint* actuator1;
	CylindricalJoint* actuator2;
	CylindricalJoint* actuator3;

	JointController controller1;
	JointController controller2;
	JointController controller3;
};


ClimbingFish::ClimbingFish(ClimbingFishParams& params)
	: body(new Body(*gScene, NxVec3(0, 0, 4))) {
	for(int i = 0; i < 4; ++i)
		limbs.push_back(new TripartiteLimb(*gScene, *body, i, params));
}

ClimbingFish::~ClimbingFish() {
	for(int i = 0; i < limbs.size(); ++i)
		delete limbs[i];
	delete body;
}

NxVec3 ClimbingFish::getPosition() { return body->getZUpGlobalPose().t; }

void ClimbingFish::update(float simulationTime) {
	for(int i = 0; i < limbs.size(); ++i)
		limbs[i]->update(simulationTime);
}







float evaluate(GAGenome& genome) {
	EvolutionHarness* evoHarness = static_cast<EvolutionHarness*>(genome.userData());
	if(!evoHarness->isRunning())
		return 0;

	if(gPhysicsSDK) 
		terminatePhysics();
	initPhysics();

	ClimbingFishParams p;
	evoHarness->convertParams(genome, p);
	ClimbingFish climbingFish(p);

	int runFrames=2500;
	NxVec3 firstPos=climbingFish.getPosition();
	NxVec3 lastPos=firstPos;
	double distSum=0.0;
	bool cheat=false; //for detecting simulator flaws or other unwanted behavior
	double simTime=0;

	int f;
	for(f=0; f<runFrames && !cheat; f++) {
		bool freeze=false;
		if(glfwGetKey(GLFW_KEY_F3)) freeze=true;
		if( glfwGetKey(GLFW_KEY_ESC) ) {
			evoHarness->stop();
			break; 
		}

		if(freeze && f>0) //freeze hack, ugly
			f--;

		climbingFish.update((float)simTime);
		setFollowTargetAndPanning(Vec3(lastPos.x,lastPos.y,lastPos.z),0);
		evoHarness->updateGraphicsAndPhysics(freeze);
		if(!freeze)
			simTime+=evoHarness->m_timestep;
		lastPos=climbingFish.getPosition();

		if (!climbingFish.body->getZAxis().sameDirection(NxVec3(0, 0, 1)))
			cheat = true;

		/*for(int i=0;i<3;i++) {
			if(climbingFish.m_touchSensors[i]==true) {
				cheat=true;
				break;
			}
		}*/
	} //end eval loop

	NxVec3 diff=climbingFish.getPosition()-firstPos;
	terminatePhysics();

	float fitnessValue=diff.magnitude();
	fitnessValue=fitnessValue*60.0f/runFrames;

	if(cheat) {
		printf(" cheat!");
		fitnessValue=0;
	}
	printf("\t\t\tfit: %f\n",fitnessValue);
	return fitnessValue;
}

ClimbingFishEvolutionHarness::ClimbingFishEvolutionHarness(int seed) : EvolutionHarness() {
	setSimulatorTimeStep(1/30.0f);
	GARandomSeed(seed);
	createGA(ClimbingFishParams(), evaluate, "ClimbingFishSettings.txt");
}

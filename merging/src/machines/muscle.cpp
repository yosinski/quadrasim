#include "muscle.h"
#include "../base/system.h"

//////////////////////////////////////////////////////////////////////////
// muscle implemented with cloth
//////////////////////////////////////////////////////////////////////////
ClothMuscle::ClothMuscle(NxActor* actor1, NxVec3& localPos1, NxActor* actor2, NxVec3& localPos2,float radius,int heightDivs,int aroundDivs)
{
	m_minPressure=1.0f;
	m_maxPressure=1.5f; //max = min + max now
	//m_maxPressure=20.0f; //max = min + max now

	//DISSE SETTINGENE FUNKER GANSKE OK I SW! BRUK DE FOR EVO RUN!

	NxVec3 globalPos1,globalPos2;
	actor1->getGlobalPose().multiply(localPos1,globalPos1);
	actor2->getGlobalPose().multiply(localPos2,globalPos2);

	NxClothDesc clothDesc;
	clothDesc.globalPose.t = globalPos1;
	//find orientation of muscle, is there an easier way?
	//now calculating axis-angle information
	NxVec3 dir=globalPos2-globalPos1;
	dir.normalize();
	NxVec3 up(0,1,0); //"up" vetcctor as starting point
	NxVec3 axis;
	if(dir.equals(up,0.0001f))
		axis.set(0,0,1);
	else
		axis=up.cross(dir); //using "up" vector  

	float angle=acos(up.dot(dir))/PI*180.0f;
	clothDesc.globalPose.M.fromQuat(NxQuat(angle,axis));

	//clothDesc.hierarchicalSolverIterations=3; //default: 2
	clothDesc.dampingCoefficient = 1.0f; //default:0.5
	//clothDesc.density=1.0f;
	//clothDesc.density=20.0f; //this may be the key param if muscles are too weak!!
	//clothDesc.density=1.0f; //gpu does not handle high density
	clothDesc.density=10.0f; //this may be the key param if muscles are too weak!!
	//clothDesc.friction = 0.5f;
	clothDesc.pressure = 1.1f; //slightly inflated for static pose
	clothDesc.solverIterations = 8; //default: 5
	//clothDesc.solverIterations = 35; //default: 5
	//clothDesc.bendingStiffness=0.8f;
	//clothDesc.stretchingStiffness=0.9999f;
	clothDesc.attachmentResponseCoefficient=1.0f;
	clothDesc.flags |= NX_CLF_PRESSURE;
	//clothDesc.flags |= NX_CLF_GRAVITY; //not sure if good or not
	clothDesc.flags |= NX_CLF_BENDING;
	clothDesc.flags |= NX_CLF_BENDING_ORTHO; //more realistic, stiffer bending
	clothDesc.flags |= NX_CLF_DAMPING;
	clothDesc.flags |= NX_CLF_COMDAMPING;
	clothDesc.flags |= NX_CLF_COLLISION_TWOWAY; //cloth can also pull other things
	clothDesc.flags |= NX_CLF_DISABLE_COLLISION; //collision with other objects.. can still pull objects

	if(gHardwareSimulation)
		clothDesc.flags |= NX_CLF_HARDWARE;

	float height=globalPos1.distance(globalPos2);
	m_clothMuscle = new MyCloth(gScene,clothDesc,height,radius,heightDivs,aroundDivs); //use handmade muscle constructor
	if(NULL==m_clothMuscle)
		systemError("cloth muscle could not be created");
	cloths.push_back(m_clothMuscle);

	//now also attach it to colliding shapes!
	m_clothMuscle->getNxCloth()->attachToCollidingShapes(NX_CLOTH_ATTACHMENT_TWOWAY);
}





void ClothMuscle::setPull(float val)
{
	m_clothMuscle->getNxCloth()->setPressure(val*m_maxPressure+m_minPressure);
}



//////////////////////////////////////////////////////////////////////////
// muscle implemented with distance joint
//////////////////////////////////////////////////////////////////////////
DistanceJointMuscle::DistanceJointMuscle(NxActor* actor1, NxVec3& localPos1, NxActor* actor2, NxVec3& localPos2,float stretchFactor)
{
	//m_stretchAndContractFactor=0.05f;
	m_stretchAndContractFactor=stretchFactor;
	m_stretchAndContractLength=0.0f;
	m_useAbsoluteStretchLength=false;

	NxDistanceJointDesc distDesc;    
	distDesc.actor[0] = actor1;    
	distDesc.actor[1] = actor2;    
	distDesc.localAnchor[0]=localPos1;
	distDesc.localAnchor[1]=localPos2;

	//need to find global coords to compute current distance
	NxVec3 globalPos1,globalPos2;
	actor1->getGlobalPose().multiply(localPos1,globalPos1);
	actor2->getGlobalPose().multiply(localPos2,globalPos2);
	m_originalLength=globalPos1.distance(globalPos2);
	distDesc.maxDistance=m_originalLength;
	distDesc.minDistance=m_originalLength; 

	distDesc.flags=NX_DJF_MIN_DISTANCE_ENABLED | NX_DJF_MAX_DISTANCE_ENABLED; //Enforce a fixed distance.    

	m_jointMuscle=(NxDistanceJoint*)gScene->createJoint(distDesc); 
}

void DistanceJointMuscle::setPull(float val)
{
	val=clamp(val,0,1);
	val=val*2-1.0f; //from -1 to +1
	float dist;
	if(m_useAbsoluteStretchLength) {
		//dist=m_originalLength+val*m_stretchAndContractLength;
		dist=m_originalLength-val*m_stretchAndContractLength; //val 0 (-1 after scale) should give most extension, not contraction
	}
	else {
		//dist=m_originalLength+val*m_stretchAndContractFactor*m_originalLength;
		dist=m_originalLength-val*m_stretchAndContractFactor*m_originalLength;
	}

	NxDistanceJointDesc desc;
	m_jointMuscle->saveToDesc(desc);
	desc.minDistance=dist;
	desc.maxDistance=dist;
	m_jointMuscle->loadFromDesc(desc);
}

float DistanceJointMuscle::getStretchLength()
{
	if(m_useAbsoluteStretchLength)
		return m_stretchAndContractLength;
	else
		return m_stretchAndContractFactor*m_originalLength;
}

void DistanceJointMuscle::setStretchLength(float stretchLength)
{
	m_stretchAndContractLength=stretchLength;
	m_useAbsoluteStretchLength=true;
}


//////////////////////////////////////////////////////////////////////////
// muscle implemented with distance joint and spring 
//////////////////////////////////////////////////////////////////////////
SpringMuscle::SpringMuscle(NxActor* actor1, NxVec3& localPos1, NxActor* actor2, NxVec3& localPos2,float stretchFactor, float springValue,float damperValue)
: DistanceJointMuscle(actor1,localPos1,actor2,localPos2,stretchFactor)
{
	NxDistanceJointDesc desc;
	m_jointMuscle->saveToDesc(desc);
	desc.flags|=NX_DJF_SPRING_ENABLED;
	NxSpringDesc sd;
	//targetValue field not used
	sd.damper=damperValue;
	sd.spring=springValue;
	desc.spring=sd;
	m_jointMuscle->loadFromDesc(desc);



}
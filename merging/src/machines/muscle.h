#ifndef _MUSCLE_H
#define _MUSCLE_H

#include "../physics.h"
#include "../physx/MyCloth.h"

//////////////////////////////////////////////////////////////////////////
// muscle base class / interface
//////////////////////////////////////////////////////////////////////////
class Muscle 
{
public:
	virtual void setPull(float val)=0;
};

//////////////////////////////////////////////////////////////////////////
// muscle implemented with cloth
//////////////////////////////////////////////////////////////////////////
class ClothMuscle : public Muscle
{
public:
	//ClothMuscle(NxVec3& startPos,NxVec3& endPos,float radius,int heightDivs,int aroundDivs);
	ClothMuscle(NxActor* actor1, NxVec3& localPos1, NxActor* actor2, NxVec3& localPos2,float radius,int heightDivs,int aroundDivs);
	virtual void setPull(float val);
	MyCloth* m_clothMuscle;
	float m_minPressure;
	float m_maxPressure;
};

//////////////////////////////////////////////////////////////////////////
// muscle implemented with distance joint
//////////////////////////////////////////////////////////////////////////
class DistanceJointMuscle : public Muscle
{
public:
	DistanceJointMuscle(NxActor* actor1, NxVec3& localPos1, NxActor* actor2, NxVec3& localPos2,float stretchFactor);
	virtual void setPull(float val);
	float getStretchLength();
	void setStretchLength(float stretchLength); //stretchLength: additional stretching length
	NxDistanceJoint* m_jointMuscle;
	float m_originalLength;
	float m_stretchAndContractFactor;
	float m_stretchAndContractLength;
	bool m_useAbsoluteStretchLength;

};

//////////////////////////////////////////////////////////////////////////
// muscle implemented with distance joint and spring 
//////////////////////////////////////////////////////////////////////////
class SpringMuscle : public DistanceJointMuscle
{
public:
	SpringMuscle(NxActor* actor1, NxVec3& localPos1, NxActor* actor2, NxVec3& localPos2,float stretchFactor, float springValue,float damperValue);
	//virtual void setPull(float val);
};





#endif
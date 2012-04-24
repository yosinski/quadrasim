#ifndef _JOINT_H
#define _JOINT_H

#include "part.h"
#include "NxPhysics.h"

class Joint
{
public:
	NxJoint *physicsEquivalent;
	//possible actuator

	
	Part *part1,*part2; //link to connecting parts


	Joint() {
		part1=part2=NULL;
		physicsEquivalent=NULL;
	};

	Joint(Part *p1, Part *p2,NxJoint *joint) {
		part1=p1;
		part2=p2;
		physicsEquivalent=joint;
	}

	void update(double time) { };

};



#endif

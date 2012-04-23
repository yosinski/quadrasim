#ifndef MACHINE_H
#define MACHINE_H

#include "../part.h"

class Machine
{
public:
	//could consider giving default implems of these?
	virtual void update(float simulationTime)=0;
	virtual	Part* getCentralPart(void)=0; 

};


#endif
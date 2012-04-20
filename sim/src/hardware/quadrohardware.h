#ifndef _QUADROHARDWARE_H
#define _QUADROHARDWARE_H

#include "axservo.h"
#include "../machines/quadrobot.h"

//todo: 
//readback of servo positions (now and then if too comms heavy)
//interpolate starting positions so not too abrupt starting

//Overload shutdown problem:
//set "moving speed" in the init - this is the max speed of servos, to reduce chances of overload shutdown(?)
// - AX18: ca 256 (25% speed)
// - AX12: ca 512 (50% speed. but ax12 is slower)
//Is it also possible to set max torque? How does the overload error work?
//"Torque limit": does not control overload(?) but gives the max torque to apply to get to a position. Can be adjusted down to have "compliance"

//Other cause for overload could be not delivering enough current? PSU: 5A. AX12 0.9A AX18 2.2A (max values)

//det virker som overload inntreffer når den prøver å loade men ikke får nok strøm eller noe annet.
//torque limit gjør mer til at den ikke prøver, men gir ikke overload i seg selv(?)


class QuadroHardware : public Quadrobot
{
public:
	
	//setup servos in constructor?
	//or always assume servos set up (pass dxldevice object)
	//update func
	QuadroHardware(QuadroParams* params=NULL);
	~QuadroHardware();
	void update(float time);
	void enableHWLogging(bool enable);

protected:

	//servo references

	DynamixelDevice dxlDevice;

	FILE* hwLogFile;
	bool logHW;
};



#endif

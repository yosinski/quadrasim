#ifndef _AXSERVO_H
#define _AXSERVO_H
#include <vector>

//sync write implemented but not tested yet

struct AXServo
{
	AXServo() {
		id=-1;
		model=-1;
		version=-1;
	}
	//int init(DynamixelInterface* dxl,int broadcastId);
	int id;
	int model;
	int version;
};

struct DynamixelDevice
{
	bool init(int baudSetting=1); //1-255, refer to dynamixel manual for different settings
	void shutdown();
	void displayError();
	void setPosition(int servoId,int goalPosition);
	void setPositions(int* goalPositions, int numServos);
	int getPosition(int servoId);
	int baud;
	//get servo object from here? or pass the device to servo object for init?

	std::vector<AXServo> servos;
};




#endif


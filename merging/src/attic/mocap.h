#ifndef _MOCAP_H
#define _MOCAP_H

#include <vector>
#include "base/Vec3.h"


/*struct MocapBot
{
	Mocap mocapData;
	std::vector<Vec3> neckPos;
	//trenger matriser...
};*/


struct Mocap
{
	int m_nFrames;
	int m_nMarkers;
	int m_frequency;
	float m_endTime;



	std::vector<std::vector<Vec3>> m_positions;
	std::vector<std::vector<float>> m_absVel;
	std::vector<std::vector<float>> m_absAcc;

	void loadFromFile(const char* fileName);
	Vec3 getPosition(float time,int marker);
	float getAbsVel(float time,int marker);
	void scaleData(float val);

	void saveBinary(const char* fileName); //for 64k
	//void saveIncludeFile(const char* fileName); //for 64k
	void saveIncludeFile(const char* fileName,int dataType=0); //for 64k
	void processDataForIntro(); //for 64k
	Vec3 scaleFactors;


};



#endif
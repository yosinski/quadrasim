#include "quadrohardware.h"

#include "axservo.h"



QuadroHardware::QuadroHardware(QuadroParams* params/* =NULL */)
{
	init(params);
	//add some error checking etc. later
	dxlDevice.init();
	hwLogFile=NULL;
	logHW=false;
}

QuadroHardware::~QuadroHardware()
{
	dxlDevice.shutdown();
}


void QuadroHardware::update(float time)
{
	updateControl(time);

	if(logHW && hwLogFile) fprintf(hwLogFile,"%.3f ",time); 


	int goalPositions[NUM_JOINTS]; //for sync write
	for(unsigned i=0;i<m_jointTargets.size();i++) {
		float angularPos=m_jointTargets[i];

		std::pair<float,float> lims;
		lims=m_jointLimitAngles[i];

		//if(i==0) printf("servo %d control angle: %.2f ",i,angularPos);
		if(angularPos<lims.first) angularPos=lims.first; //limit checking
		if(angularPos>lims.second) angularPos=lims.second; //limit checking
		//if(i==0) printf("limits: %.2f %.2f, result: %.2f\n",lims.first,lims.second,angularPos);

		int servoAngle=(int)angleToServo(angularPos);
		goalPositions[i]=servoAngle; //for sync write
		//if(i==0) printf("writing %d to servo\n",servoAngle);
		//dxlDevice.setPosition(i,servoAngle); //remove if sync write

		if(logHW && hwLogFile) {
			fprintf(hwLogFile,"%d ",servoAngle); 
			int actualServoAngle=dxlDevice.getPosition(i);
			fprintf(hwLogFile,"%d ",actualServoAngle); 
		}
	}

	dxlDevice.setPositions(goalPositions,m_jointTargets.size()); //for sync write

	if(logHW && hwLogFile) fprintf(hwLogFile,"\n"); 

}

void QuadroHardware::enableHWLogging(bool enable)
{
	//later: add option for logging on specific frames - auto-swith off ?
	std::string logFileName="log_hw.txt";
	logHW=enable;
	if(hwLogFile==NULL) {
		printf("opening log file %s for writing\n",logFileName.c_str());
		if(!(hwLogFile=fopen(logFileName.c_str(),"w"))) {
			printf("could not open log file %s for writing\n",logFileName.c_str());
			logHW=false;
		}
	}
	fprintf(hwLogFile,"# time (joint_commanded joint_actual_pos)x%d (pos.x pos.y pos.z will come later when mocap added)\n",NUM_JOINTS);
}

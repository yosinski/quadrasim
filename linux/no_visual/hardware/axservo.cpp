#include "../base/system.h"
#include "axservo.h"
#include "../hardware/dynamixel.h"
#include <stdio.h>

// Control table address
#define P_MODEL_NUMBER_L	0	// Model number low byte
#define P_MODEL_NUMBER_H	1	// Model number high byte
#define P_VERSION			2	// Firmware version
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46


int deviceIndex = 1;
int baudnum = 1;

bool DynamixelDevice::init(int baudSetting)
{
	if( dxl_initialize(deviceIndex,baudnum ) == 0 ) {
		printf("could not initialize usb2dynamixel\n");
		return false;
	}
	printf( "usb2dynamixel initialized\n" );

	dxl_set_baud( baudSetting );

	for(int i=0;i<BROADCAST_ID;i++) {
		dxl_ping(i);
		if(dxl_get_result()==COMM_RXSUCCESS) {
			AXServo servo;
			servo.id=i;
			int model=dxl_read_word(i,P_MODEL_NUMBER_L);
			if( dxl_get_result()==COMM_RXSUCCESS)
				servo.model=model;
			int version=dxl_read_byte(i,P_VERSION);
			if(dxl_get_result()==COMM_RXSUCCESS)
				servo.version=version;
			servos.push_back(servo);
			printf( "id:%d, model:%d, version:%d\n",servo.id,servo.model,servo.version);
		}
		else {
			//displayError();
			//do nothing
		}
	}
	printf("%d servos found\n",servos.size());
	return true;
}


void DynamixelDevice::shutdown()
{
	dxl_terminate();

}


void DynamixelDevice::displayError()
{
	switch(dxl_get_result()) {
		case COMM_TXFAIL: printf( "COMM_TXFAIL" ); break;
		case COMM_TXERROR: printf( "COMM_TXERROR" ); break;
		case COMM_RXFAIL: printf( "COMM_RXFAIL" ); break;
		case COMM_RXWAITING: printf( "COMM_RXWAITING" ); break;
		case COMM_RXTIMEOUT: printf( "COMM_RXTIMEOUT" ); break;
		case COMM_RXCORRUPT: printf( "COMM_RXCORRUPT" ); break;
	}
}

void DynamixelDevice::setPosition(int servoId,int goalPosition)
{
	//dxl_write_word(servoId,P_GOAL_SPEED_L, 0 ); //is this necessary? try to remove
	dxl_write_word(servoId,P_GOAL_POSITION_L,goalPosition);
	displayError(); //this one does not slow down, but dxl_write_word waits for a reply. txrx_packet. Could swap with tx_packet only
}

int DynamixelDevice::getPosition(int servoId)
{
	int pos=dxl_read_word(servoId,P_PRESENT_POSITION_L);
	return pos;
}

void DynamixelDevice::setPositions(int* goalPositions, int numServos)
{
	//for now this one assumes the servos have IDs like this: [0..numServos-1]
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, 2); //what is this?
	for(int i=0;i<numServos;i++) {
		dxl_set_txpacket_parameter(2+3*i, i); //servo ID
		int goalPos=goalPositions[i];
		printf("[%d]:%d ",i,goalPos);
		dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(goalPos));
		dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(goalPos));
	}
	printf("\n");
	dxl_set_txpacket_length((2+1)*numServos+4);
	dxl_txrx_packet();
	//could possibly do some error checking after this
}
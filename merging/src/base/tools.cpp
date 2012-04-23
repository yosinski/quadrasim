#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "tools.h"


float randRangef(float minVal,float maxVal)
{
	return (((float)rand())/RAND_MAX)*(maxVal-minVal)+minVal;
}


float linearEnvelope(float val,float t0,float t1,float t2,float t3)
{
	//      ____          1
	//     /    \
	//    /      \_____   0
	//  t:0 1  2  3

	float res=0;
	if(  val >= t0 && val < t1 ) 
		res=(val-t0)/(t1-t0);
	else if( val >= t1 && val < t2 )
		res=1.0f;
	else if( val >= t2 && val < t3 )
		res=(1.0f-(val-t2)/(t3-t2));
	else
		res=0.0f;

	return res;
}


float sineEnvelope(float val,float t0,float t1,float t2,float t3)
{
	//      ____          1
	//     /    \
	//    /      \_____   0
	//  t:0 1  2  3

	float res=0;
	if(  val >= t0 && val < t1 ) 
		res=sin((val-t0)/(t1-t0)*PI-PI/2)*0.5f+0.5f;
	else if( val >= t1 && val < t2 )
		res=1.0f;
	else if( val >= t2 && val < t3 )
		res=sin((val-t2)/(t3-t2)*PI+PI/2)*0.5f+0.5f;
	else
		res=0.0f;

	return res;
}


#ifndef _TOOLS_H
#define _TOOLS_H

#ifndef PI
#define PI 3.1415926535f
#endif


float randRangef(float minVal,float maxVal);
float linearEnvelope(float val,float t0,float t1,float t2,float t3);
float sineEnvelope(float val,float t0,float t1,float t2,float t3);



#endif
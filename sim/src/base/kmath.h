#ifndef _KMATH_H
#define _KMATH_H

#ifndef PI 
#define PI 3.14159265358979323846
#endif

float clamp(float val, float minVal, float maxVal);

struct Point3D
{
	float x,y,z;

	Point3D(float x,float y,float z) {
		this->x=x;
		this->y=y;
		this->z=z;
	}
	Point3D() {
		this->x=0;
		this->y=0;
		this->z=0;
	}

};




#endif
#include "Vec3.h"
#include <math.h>

Vec3::Vec3(void)
{
	x = 0;
	y = 0;
	z = 0;
}

Vec3::Vec3(float x,float y,float z) 
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Vec3::Vec3(float* v)
{
	x=v[0];
	y=v[1];
	z=v[2];
}

Vec3::Vec3(const Vec3 *from, const Vec3 *to)
{
	x = to->x - from->x;
	y = to->y - from->y;
	z = to->z - from->z;
}

Vec3::Vec3(const Vec3 &source)
{
	this->x = source.x;
	this->y = source.y;
	this->z = source.z;
}

void Vec3::set(float x, float y, float z)
{
	this->x=x;
	this->y=y;
	this->z=z;
}

void Vec3::scale(float fact)
{
	x*=fact;
	y*=fact;
	z*=fact;
}

void Vec3::normalize(void) {
	float d= x*x+y*y+z*z;

	if ( d <0.00001f ) {
		d=1.0;	
	}

	d=(float)(1.0/sqrt(d));
	x*=d;
	y*=d;
	z*=d;	
}

void Vec3::add(Vec3 *vect)
{
	x+=vect->x;
	y+=vect->y;
	z+=vect->z;
}


float Vec3::dotProduct(Vec3 *vect)
{
  return x * vect->x + y * vect->y + z * vect->z;
}

float Vec3::length(void)
{
  return (float)sqrt(x * x + y * y + z * z);
}

void Vec3::crossProduct(Vec3 *a,Vec3 *b)
{
	x= a->y*b->z - a->z*b->y;
	y= a->z*b->x - a->x*b->z;
	z= a->x*b->y - a->y*b->x;
}

Vec3 Vec3::interpolate(Vec3 *vect,float t)
{
	Vec3 res;
	res.x=t*vect->x + (1-t)*this->x;
	res.y=t*vect->y + (1-t)*this->y;
	res.z=t*vect->z + (1-t)*this->z;
	return res;
}

Vec3 Vec3::distance( const Vec3 &vec ) {
	return	Vec3(	sqrt( (x - vec.x)*(x - vec.x) ),	
						sqrt( (y - vec.y)*(y - vec.y) ),
						sqrt( (z - vec.z)*(z - vec.z) ));
}

Vec3 Vec3::distanceSqr( const Vec3 &vec ) {
	return	Vec3(	(x - vec.x)*(x - vec.x),	
						(y - vec.y)*(y - vec.y),
						(z - vec.z)*(z - vec.z));
}

//-----------------------------------------------------------------------------
// Operators

void Vec3::operator=(const Vec3 &operand) 
{
	x = operand.x;
	y = operand.y;
	z = operand.z;
}

Vec3 Vec3::operator+(const Vec3 &operand)  
{
	return Vec3( x + operand.x, y + operand.y, z + operand.z );
}

void Vec3::operator+=(const Vec3 &operand) 
{
	x += operand.x;
	y += operand.y;
	z += operand.z;
}

Vec3 Vec3::operator-(const Vec3 &operand) 
{
	return Vec3( x - operand.x, y - operand.y, z - operand.z );	
}

void Vec3::operator-=(const Vec3 &operand) 
{
	x -= operand.x;
	y -= operand.y;
	z -= operand.z;
}

Vec3 Vec3::operator*(float factor) 
{
	return Vec3( x * factor, y * factor, z * factor );
}

void Vec3::operator*=(float factor) 
{
	x *= factor;
	y *= factor;
	z *= factor;
}

Vec3 Vec3::operator*(const Vec3 &operand) 
{
	return Vec3( x * operand.x, y * operand.y, z * operand.z );
}

Vec3 Vec3::operator/(const Vec3 &operand) 
{
	Vec3 retVal( 0, 0, 0);
	if ( operand.x != 0 ) retVal.x = x / operand.x;
	if ( operand.y != 0 ) retVal.y = y / operand.y;
	if ( operand.z != 0 ) retVal.z = z / operand.z;


	return retVal;
}

void Vec3::operator/=(const Vec3 &operand) 
{	
	if ( operand.x != 0 ) x /= operand.x;
	if ( operand.y != 0 ) y /= operand.y;
	if ( operand.z != 0 ) z /= operand.z;
}

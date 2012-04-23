//based on pp engine
//by Jan-Harald Fredriksen and Kyrre Glette

#ifndef _VEC3_H
#define _VEC3_H

class Vec3 {
public:	
	float x, y, z;

	Vec3(void);
	Vec3(float x,float y,float z);
	Vec3(float* v);
	Vec3(const Vec3 *from, const Vec3 *to);	
	Vec3(const Vec3 &source);

	void operator=(const Vec3 &operand);

	Vec3 operator+(const Vec3 &operand);
	void operator+=(const Vec3 &operand);
	
	Vec3 operator-(const Vec3 &operand);
	void operator-=(const Vec3 &operand);
	
	Vec3 operator*(float factor);
	void operator*=(float factor);

	// component-wise multiplication
	Vec3 operator*(const Vec3 &operand);

	// component-wise division
	Vec3 operator/(const Vec3 &operand);
	void operator/=(const Vec3 &operand);
	
	Vec3 distance( const Vec3 &vec );
	Vec3 distanceSqr( const Vec3 &vec );

	void set(float x,float y,float z);
	void scale(float fact);
	void normalize(void);
	void add(Vec3 *vect);
	float dotProduct(Vec3 *vect);
	float length(void);
	void crossProduct(Vec3 *a,Vec3 *b);
	Vec3 interpolate(Vec3 *vect,float t);
};

#endif

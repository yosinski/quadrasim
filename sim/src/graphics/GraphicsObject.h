#ifndef _GRAPHICSOBJECT_H
#define _GRAPHICSOBJECT_H

#include "../base/Vec3.h"
#include "shaders.h"


//classes for rendering the basic objects
class GraphicsObject
{
public:
	virtual void render(void)=0;
};



class BoxGraphicsObject : public GraphicsObject
{
protected:
	float width,depth,height;
public:
	BoxGraphicsObject(float width,float depth,float height);
	void render();
};

class SphereGraphicsObject : public GraphicsObject
{
private:
	float radius;
	static Vec3 *verts;
	static Vec3 *normals;
	static int nVerts;
public:
	SphereGraphicsObject(float radius);
	void render();
	static void initVertices(int steps);
};

class CapsuleGraphicsObject : public GraphicsObject
{
protected:
	float radius;
	float length;
	SphereGraphicsObject *sphere;
	static Vec3 *verts;
	static Vec3 *normals;
	static int nVerts;
public:
	CapsuleGraphicsObject(float radius, float length);
	void render();
	static void initVertices(int iterations);
};


#endif
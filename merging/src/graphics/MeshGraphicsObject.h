#ifndef _MESHGRAPHICSOBJECT_H
#define _MESHGRAPHICSOBJECT_H

#include "glstuff.h"
#include "../base/Vec3.h"
#include "Vertex.h"
#include "GraphicsObject.h"
#include "material.h"



//structure only for data loading
struct Face
{
	unsigned int v0,v1,v2;
	unsigned int n0,n1,n2;
	unsigned int t0,t1,t2;
	bool hasNormal;
	bool hasTexCoords;
};


class MeshGraphicsObject : public GraphicsObject
{
protected:
	int numFaces;
	Vertex *vertices;
	//now just having 3 verts for every poly, dropping indexing. Otherwise slightly complicated with sharing normals, tcoords
	//unsigned int* indices;
	
	//Material *m_mat;

	GLuint vboRef;

	void loadMesh(const char *fileNameNoExtension);
	void loadOBJ(const char *fileName);
	void dumpToBinOBJ(const char *fileName);
	void loadBinOBJ(const char *fileName);
	void drawTriangles();
	//void calculateNormals(); //deprecated

public:
	float transform[16]; //modify this if mesh size does not correspond to physics equivalent
	MeshGraphicsObject(const char* fileName, float scale=1.0f, float xTrans=0, float yTrans=0, float zTrans=0, float xRot=0, float yRot=0, float zRot=0);
	void setTransform(float scale,float xTrans=0, float yTrans=0, float zTrans=0, float xRot=0, float yRot=0, float zRot=0);
	void getExtents(Vec3& minVect, Vec3& maxVect);
	void render();
	Material *m_mat;
};




#endif
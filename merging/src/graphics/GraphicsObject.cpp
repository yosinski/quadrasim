#include "GraphicsObject.h"

#include "../base/system.h"
#include "graphics.h"
#include "../base/tools.h"
#include "glstuff.h"
#include <Cg/cg.h>
#include <Cg/cgGL.h>
#include <math.h>
#include <stdio.h>
#include <crtdbg.h>



//simple "unit" cylinder only defined around y axis, scale and rotate later
//length=1 (-0.5,+0.5)
Vec3* CapsuleGraphicsObject::verts;
Vec3* CapsuleGraphicsObject::normals;
int CapsuleGraphicsObject::nVerts;
void CapsuleGraphicsObject::initVertices(int steps)
{
	int nFaces=2*steps;
	verts=new Vec3[nFaces*3];
	normals=new Vec3[nFaces*3];
	_ASSERTE(verts);
	_ASSERTE(normals);
	nVerts=nFaces*3;
	printf("cylinder: %d steps, %d faces created\n",steps,nFaces);

	Vec3 p0,p1; //lower disc
	Vec3 p2,p3; //upper disc

	for (int i=0;i<steps;i++) {
		float theta = i * 2.0f*PI / steps;
		float theta2 = (i+1) * 2.0f*PI / steps;

		p0.x = cos(theta);
		p0.y = -0.5f;
		p0.z = sin(theta);
		p1.x = cos(theta2);
		p1.y = -0.5f;
		p1.z = sin(theta2);

		p2.x = cos(theta);
		p2.y = 0.5f;
		p2.z = sin(theta);
		p3.x = cos(theta2);
		p3.y = 0.5f;
		p3.z = sin(theta2);

		//remember right handed coord sys
		verts[i*6+0]=p0; 
		verts[i*6+1]=p2;
		verts[i*6+2]=p1;
		verts[i*6+3]=p1;
		verts[i*6+4]=p2;
		verts[i*6+5]=p3;

		//normals:
		p0.y=0; p1.y=0; p2.y=0; p3.y=0;
		normals[i*6+0]=p0;
		normals[i*6+1]=p2;
		normals[i*6+2]=p1;
		normals[i*6+3]=p1;
		normals[i*6+4]=p2;
		normals[i*6+5]=p3;
	}
}


void CapsuleGraphicsObject::render()
{
	glShadeModel(GL_SMOOTH);

	if(g_shaderSys)
		g_shaderSys->enableProfiles();
	if(g_defaultVertexShader) {
		g_defaultVertexShader->bind();
	}
	if(g_defaultFragmentShader)
		g_defaultFragmentShader->bind();

	glPushMatrix();
	glEnable(GL_NORMALIZE);
	glScalef(radius,length,radius);
	if(g_defaultVertexShader) {
		cgGLSetParameter4f(g_defaultVertexShader->getParameter("lightPos"), g_lightPos.x,g_lightPos.y,g_lightPos.z,1); //move to bind place
		cgGLSetMatrixParameterfc(g_defaultVertexShader->getParameter("view"),g_viewMat);
		cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldViewProj"), CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldView"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgUpdateProgramParameters(g_defaultVertexShader->getProgram());
	}

	glColor3f(1,1,1);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,verts);
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT,0,normals);
	glDrawArrays(GL_TRIANGLES,0,nVerts);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisable(GL_NORMALIZE);
	glPopMatrix();

	//sphere caps
	glPushMatrix();
	glTranslatef(0,length/2,0);
	sphere->render();
	glPopMatrix();
	glPushMatrix();
	glTranslatef(0,-length/2,0);
	sphere->render();
	glPopMatrix();

	if(g_shaderSys)
		g_shaderSys->disableProfiles();
}

CapsuleGraphicsObject::CapsuleGraphicsObject( float radius, float length )
{
	this->radius=radius;
	this->length=length;
	sphere = new SphereGraphicsObject(radius);
	_ASSERTE(sphere);
}


//BOX

BoxGraphicsObject::BoxGraphicsObject(float width,float depth,float height)
{
	this->width=width;
	this->depth=depth;
	this->height=height;
}


float boxVerts[] = {
	1,1,1,  -1,1,1,  -1,-1,1,  1,-1,1,
	1,1,1,  1,-1,1,  1,-1,-1,  1,1,-1,        
	1,1,1,  1,1,-1,  -1,1,-1,  -1,1,1,        
	-1,1,1,  -1,1,-1,  -1,-1,-1,  -1,-1,1,    
	-1,-1,-1,  1,-1,-1,  1,-1,1,  -1,-1,1,    
	1,-1,-1,  -1,-1,-1,  -1,1,-1,  1,1,-1
};   
float boxNormals[] = {
	0,0,1,  0,0,1,  0,0,1,  0,0,1,     
	1,0,0,  1,0,0,  1,0,0, 1,0,0,      
	0,1,0,  0,1,0,  0,1,0, 0,1,0,      
	-1,0,0,  -1,0,0, -1,0,0,  -1,0,0,  
	0,-1,0,  0,-1,0,  0,-1,0,  0,-1,0, 
	0,0,-1,  0,0,-1,  0,0,-1,  0,0,-1
};


void BoxGraphicsObject::render(void)
{
	if(g_shaderSys)
		g_shaderSys->enableProfiles();
	if(g_defaultVertexShader) {
		g_defaultVertexShader->bind();
	}
	if(g_defaultFragmentShader)
		g_defaultFragmentShader->bind();


	glPushMatrix();
	glEnable(GL_NORMALIZE);
	glScalef(width,height,depth);

	if(g_defaultVertexShader) {
		cgGLSetParameter4f(g_defaultVertexShader->getParameter("lightPos"), g_lightPos.x,g_lightPos.y,g_lightPos.z,1); //move to bind place
		cgGLSetMatrixParameterfc(g_defaultVertexShader->getParameter("view"),g_viewMat);
		cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldViewProj"), CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldView"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgUpdateProgramParameters(g_defaultVertexShader->getProgram());
	}

	glColor3f(1,1,1);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,boxVerts);
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT,0,boxNormals);
	glDrawArrays(GL_QUADS,0,24);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisable(GL_NORMALIZE);
	glPopMatrix();

	if(g_shaderSys)
		g_shaderSys->disableProfiles();
}


//SPHERE

SphereGraphicsObject::SphereGraphicsObject(float radius)
{
	this->radius=radius;
}


void SphereGraphicsObject::render()
{
	glShadeModel(GL_SMOOTH);
	if(g_shaderSys)
		g_shaderSys->enableProfiles();
	if(g_defaultVertexShader) 
		g_defaultVertexShader->bind();
	if(g_defaultFragmentShader)
		g_defaultFragmentShader->bind();

	glPushMatrix();
	glEnable(GL_NORMALIZE);
	glScalef(radius,radius,radius);
	if(g_defaultVertexShader) {
		cgGLSetParameter4f(g_defaultVertexShader->getParameter("lightPos"), g_lightPos.x,g_lightPos.y,g_lightPos.z,1); //move to bind place
		cgGLSetMatrixParameterfc(g_defaultVertexShader->getParameter("view"),g_viewMat);
		cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldViewProj"), CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldView"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgUpdateProgramParameters(g_defaultVertexShader->getProgram());
	}


	glColor3f(1,1,1);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,verts);
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT,0,normals);
	glDrawArrays(GL_TRIANGLES,0,nVerts);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisable(GL_NORMALIZE);
	glPopMatrix();

	if(g_shaderSys)
		g_shaderSys->disableProfiles();
	glShadeModel(GL_FLAT);
}


//Paul Bourke sphere source
struct Face3 {
	Vec3 p1,p2,p3;
};
//Create a triangular facet approximation to a sphere
//Return the number of facets created.
//The number of facets will be (4^iterations) * 8
int CreateNSphere(Face3 *f,int iterations)
{
	int i,it;
	double a;
	Vec3 p[6];// = {0,0,1,  0,0,-1,  -1,-1,0,  1,-1,0,  1,1,0, -1,1,0};
	p[0]=Vec3(0,0,1);
	p[1]=Vec3(0,0,-1);
	p[2]=Vec3(-1,-1,0);
	p[3]=Vec3(1,-1,0);
	p[4]=Vec3(1,1,0);
	p[5]=Vec3(-1,1,0);
	Vec3 pa,pb,pc;
	int nt = 0,ntold;

	/* Create the level 0 object */
	a = 1 / sqrt(2.0);
	for (i=0;i<6;i++) {
		//p[i].x *= a;
		//p[i].y *= a;
		p[i].x = (float)(p[i].x*a);
		p[i].y = (float)(p[i].y*a);
	}
	f[0].p1 = p[0]; f[0].p2 = p[3]; f[0].p3 = p[4];
	f[1].p1 = p[0]; f[1].p2 = p[4]; f[1].p3 = p[5];
	f[2].p1 = p[0]; f[2].p2 = p[5]; f[2].p3 = p[2];
	f[3].p1 = p[0]; f[3].p2 = p[2]; f[3].p3 = p[3];
	f[4].p1 = p[1]; f[4].p2 = p[4]; f[4].p3 = p[3];
	f[5].p1 = p[1]; f[5].p2 = p[5]; f[5].p3 = p[4];
	f[6].p1 = p[1]; f[6].p2 = p[2]; f[6].p3 = p[5];
	f[7].p1 = p[1]; f[7].p2 = p[3]; f[7].p3 = p[2];
	nt = 8;

	if (iterations < 1)
		return(nt);

	/* Bisect each edge and move to the surface of a unit sphere */
	for (it=0;it<iterations;it++) {
		ntold = nt;
		for (i=0;i<ntold;i++) {
			pa.x = (f[i].p1.x + f[i].p2.x) / 2;
			pa.y = (f[i].p1.y + f[i].p2.y) / 2;
			pa.z = (f[i].p1.z + f[i].p2.z) / 2;
			pb.x = (f[i].p2.x + f[i].p3.x) / 2;
			pb.y = (f[i].p2.y + f[i].p3.y) / 2;
			pb.z = (f[i].p2.z + f[i].p3.z) / 2;
			pc.x = (f[i].p3.x + f[i].p1.x) / 2;
			pc.y = (f[i].p3.y + f[i].p1.y) / 2;
			pc.z = (f[i].p3.z + f[i].p1.z) / 2;
			pa.normalize();
			pb.normalize();
			pc.normalize();
			f[nt].p1 = f[i].p1; f[nt].p2 = pa; f[nt].p3 = pc; nt++;
			f[nt].p1 = pa; f[nt].p2 = f[i].p2; f[nt].p3 = pb; nt++;
			f[nt].p1 = pb; f[nt].p2 = f[i].p3; f[nt].p3 = pc; nt++;
			f[i].p1 = pa;
			f[i].p2 = pb;
			f[i].p3 = pc;
		}
	}
	return(nt);
}

Vec3* SphereGraphicsObject::verts;
Vec3* SphereGraphicsObject::normals;
int SphereGraphicsObject::nVerts;

void SphereGraphicsObject::initVertices(int iterations)
{
	//The number of facets will be (4^iterations) * 8
	int nFaces=(int)pow(4.0f,iterations)*8;
	Face3 *faces=new Face3[nFaces];
	if(NULL==faces)
		systemError("could not alloc sphere");
	CreateNSphere(faces,iterations);
	printf("sphere: %d iterations, %d faces created\n",iterations,nFaces);

	verts=new Vec3[nFaces*3];
	normals=new Vec3[nFaces*3];
	nVerts=nFaces*3;

	for(int f=0;f<nFaces;f++) {
		verts[f*3+0]=faces[f].p1;
		verts[f*3+1]=faces[f].p2;
		verts[f*3+2]=faces[f].p3;

		//face normals
		/*Vec3 axis1=faces[f].p2-faces[f].p1;
		Vec3 axis2=faces[f].p3-faces[f].p1;
		Vec3 normal;
		normal.crossProduct(&axis1,&axis2);
		normal.normalize();
		*/
		
		//point normals
		normals[f*3+0]=faces[f].p1;
		normals[f*3+1]=faces[f].p2;
		normals[f*3+2]=faces[f].p3;

		//just in case
		normals[f*3+0].normalize();
		normals[f*3+1].normalize();
		normals[f*3+2].normalize();
	}
	delete faces;
}



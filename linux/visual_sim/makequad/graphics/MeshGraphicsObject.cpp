#include "../base/kmath.h"
#include "../base/system.h"
#include "graphics.h"
#include "GraphicsObject.h"
#include "MeshGraphicsObject.h"
#include "material.h"
#include <assert.h>
#include <stdio.h>
#include "glstuff.h"
#include <Cg/cg.h>
#include <Cg/cgGL.h>
#include <math.h>
#include <vector>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <string>


void MeshGraphicsObject::getExtents(Vec3& minVect, Vec3& maxVect)
{
	minVect.set(FLT_MAX,FLT_MAX,FLT_MAX);
	maxVect.set(FLT_MIN,FLT_MIN,FLT_MIN);

	Vertex *v;
	for(int i=0;i<numFaces*3;i++) {
		v=&vertices[i];
		if(v->position.x > maxVect.x) maxVect.x = v->position.x;
		if(v->position.y > maxVect.y) maxVect.y = v->position.y;
		if(v->position.z > maxVect.z) maxVect.z = v->position.z;
		if(v->position.x < minVect.x) minVect.x = v->position.x;
		if(v->position.y < minVect.y) minVect.y = v->position.y;
		if(v->position.z < minVect.z) minVect.z = v->position.z;
	}

}

void MeshGraphicsObject::loadBinOBJ(const char *fileName)
{
	//quick and dirty binary file loader
	double startTime=glfwGetTime();
	printf("loading binary object format: %s\n",fileName);
	FILE* inFile=fopen(fileName,"rb");
	assert(inFile && "could not open bin file");

	unsigned int nFaces;
	fread(&nFaces,sizeof(unsigned int),1,inFile);
	numFaces=nFaces;
	vertices=new Vertex[numFaces*3];
	fread(vertices,sizeof(Vertex),numFaces*3,inFile);
	printf("points: %d, faces %d\n",numFaces*3,numFaces);


	//unsigned int nPoints;
	//fread(&nPoints,sizeof(unsigned int),1,inFile);
	//numVertices=nPoints;
	//printf("numpoints: %d\n",numVertices);
	//vertices=new Vertex[numVertices];
	//fread(vertices,sizeof(Vertex),numVertices,inFile);
	//numFaces=nPoints/3;

	//unsigned int nFaces;
	//fread(&nFaces,sizeof(unsigned int),1,inFile);
	//numFaces=nFaces;
	//printf("numfaces: %d\n",numFaces);
	//faces=new Face[numFaces];
	//fread(faces,sizeof(Face),numFaces,inFile);

	fclose(inFile);
	printf("loading time: %.2f s\n",glfwGetTime()-startTime);
}

void MeshGraphicsObject::dumpToBinOBJ(const char *fileName)
{
	//quick and dirty binary file writer
	printf("saving to binary object format: %s\n",fileName);
	FILE* outFile=fopen(fileName,"wb");
	assert(outFile && "could not open bin file");
	//unsigned int nPoints=numVertices;
	unsigned int nFaces=numFaces;
	fwrite(&nFaces,sizeof(unsigned int),1,outFile);
	fwrite(vertices,sizeof(Vertex),numFaces*3,outFile);
	//fwrite(&nPoints,sizeof(unsigned int),1,outFile);
	//fwrite(vertices,sizeof(Vertex),numVertices,outFile);
	//fwrite(&nFaces,sizeof(unsigned int),1,outFile);
	//fwrite(faces,sizeof(Face),numFaces,outFile);
	fclose(outFile);

}

void MeshGraphicsObject::loadMesh(const char *fileName)
{
	std::string fileNameStr(fileName);
	size_t found=fileNameStr.find(".obj");
	if(found!=std::string::npos)
		fileNameStr.erase(found); //erase to end
	
	std::string binFileName=fileNameStr;
	binFileName.append(".binobj");
	printf("trying to open %s\n",binFileName.c_str());
	FILE* f=fopen(binFileName.c_str(),"rb");
	if(NULL==f) {
		//load obj file and save to binary
		std::string objFileName=fileNameStr;
		objFileName.append(".obj");
		loadOBJ(objFileName.c_str());
		dumpToBinOBJ(binFileName.c_str());
	}
	else {
		//load binary version
		fclose(f);
		loadBinOBJ(binFileName.c_str());
	}
}


void MeshGraphicsObject::loadOBJ(const char *fileName)
{
	double startTime=glfwGetTime();

	FILE *f=fopen(fileName,"r");
	if(NULL==f) systemError("could not open file");
	printf("loading .obj model %s\n",fileName);

	std::vector<Vec3> tempPoints;
	std::vector<Vec3> tempVertexNormals;
	std::vector<Vec3> tempTexCoords;
	std::vector<Face> tempFaces;

	char line[200];

	int numLines=0;
	//numVertices=0;
	numFaces=0;
	while(fgets(line,200,f)!=NULL) {
		char tempStr[10];
		sscanf(line,"%s",tempStr);
		if(strcmp(tempStr,"v")==0) {
			Vec3 p;
			sscanf(line,"v %f %f %f\n",&p.x,&p.y,&p.z);
			tempPoints.push_back(p);

		}
		else if(strcmp(tempStr,"vn")==0) {
			Vec3 p;
			sscanf(line,"vn %f %f %f\n",&p.x,&p.y,&p.z);
			tempVertexNormals.push_back(p);
		}
		else if(strcmp(tempStr,"f")==0) {
			//2 muligheter: enten 
			//f int int int   når bare ref til verts, eller
			//f int/[int]/[int]  3 ganger ex. 12//10 13//4 14//6 eller 12/11/12 etc

			//this was overkill (but interesting use of sscanf):
			//sscanf(readPos,"%[^'/']/%[^'/']/%s%n",v,t,vn,&nRead);

			//note! may be another combination not yet seen
			Face f;
			f.hasNormal=false;
			f.hasTexCoords=false;
			int v[3]={0},t[3]={0},vn[3]={0};
			if(strstr(line,"//")!=NULL) {// no texcoord indices version
				sscanf(line,"f %d//%d %d//%d %d//%d",&v[0],&vn[0],&v[1],&vn[1],&v[2],&vn[2]);
				f.hasNormal=true;
			}
			else if(strchr(line,'/')!=NULL) { //all indices version
				sscanf(line,"f %d/%d/%d %d/%d/%d %d/%d/%d",&v[0],&t[0],&vn[0],&v[1],&t[1],&vn[1],&v[2],&t[2],&vn[2]);
				f.hasNormal=true;
				f.hasTexCoords=true;
			}
			else { // int int int version
				sscanf(line,"f %d %d %d\n",&v[0],&v[1],&v[2]);
			}

			f.v0=v[0]-1;
			f.v1=v[1]-1;
			f.v2=v[2]-1;
			f.t0=t[0]-1;
			f.t1=t[1]-1;
			f.t2=t[2]-1;
			f.n0=vn[0]-1;
			f.n1=vn[1]-1;
			f.n2=vn[2]-1;
			tempFaces.push_back(f);
		}
		numLines++;
	}
	printf("read %d lines\n",numLines);
	printf("vertices %d, faces: %d\n",tempPoints.size(),tempFaces.size());
	printf("normals %d, tex coords: %d\n",tempVertexNormals.size(),tempTexCoords.size());


	//now prepare the engine mesh data from loaded mesh data
	numFaces=tempFaces.size();
	vertices=new Vertex[numFaces*3];
	for(int i=0;i<numFaces;i++) {
		Face f=tempFaces[i];

		vertices[i*3+0].position=tempPoints[f.v0];
		vertices[i*3+1].position=tempPoints[f.v1];
		vertices[i*3+2].position=tempPoints[f.v2];

		Vec3 n0,n1,n2;
		f.hasNormal=false; //TEMP!!
		if(f.hasNormal) {
			n0=tempVertexNormals[f.n0];
			n1=tempVertexNormals[f.n1];
			n2=tempVertexNormals[f.n2];
		}
		else { //if not loaded, quick calculation of face normal (no smoothing)
			Vec3 a,b;
			a=tempPoints[f.v1]-tempPoints[f.v0];
			b=tempPoints[f.v2]-tempPoints[f.v0];
			Vec3 n;
			n.crossProduct(&a,&b);
			n.normalize();
			n0=n;
			n1=n;
			n2=n;
		}
		vertices[i*3+0].normal=n0;
		vertices[i*3+1].normal=n1;
		vertices[i*3+2].normal=n2;

		Vec3 t0,t1,t2;
		if(f.hasTexCoords) {
			t0=tempTexCoords[f.t0];
			t1=tempTexCoords[f.t1];
			t2=tempTexCoords[f.t2];
		}
		vertices[i*3+0].texCoord=t0;
		vertices[i*3+1].texCoord=t1;
		vertices[i*3+2].texCoord=t2;
	}
	printf("converted to %d faces, %d points\n",numFaces,numFaces*3);



	//man kunne kanskje bare beholdt vector siden det av c++03 er garantert consecutive
	//draw( &tempPoints[0] )  gir c-style pointer
	

	printf("loading time: %.2f s\n",float(glfwGetTime()-startTime));
}

/*void MeshGraphicsObject::calculateNormals()
{
	if(numVertices<=0) return;
	printf("calculating normals\n");

	//calculate face normals - store in each point
	for(int i=0;i<numFaces;i++) {
		//calculate normals here
		Face f=faces[i];
		Vec3 a,b;
		a=vertices[f.v1].position-vertices[f.v0].position;
		b=vertices[f.v2].position-vertices[f.v0].position;
		Vec3 n;
		n.crossProduct(&a,&b);
		n.normalize();
		
		//add/contribute to shared normals
		vertices[f.v0].normal+=n;
		vertices[f.v1].normal+=n;
		vertices[f.v2].normal+=n;
	}
	for(int i=0;i<numVertices;i++) {
		vertices[i].normal.normalize();
	}
}*/

void MeshGraphicsObject::drawTriangles()
{

	glColor3f(1,1,1);

	/*//before: not VBO (7.5 fps)
	int stride=sizeof(Vertex);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,stride,&vertices->position);

	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT,stride,&vertices->normal);

	//glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	//glTexCoordPointer(3,GL_FLOAT,stride,&vertices->texCoord);*/


	//now: VBO (52 fps 7x speed increase)
	glBindBuffer(GL_ARRAY_BUFFER,vboRef);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,sizeof(Vertex),(void *)0);

	glEnableClientState(GL_NORMAL_ARRAY);
	int normalOffset=int(&vertices[0].normal)-int(&vertices[0].position);
	glNormalPointer(GL_FLOAT,sizeof(Vertex),(void *)normalOffset);

	//glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	//int texCoordOffset=int(&vertices[0].texCoord)-int(&vertices[0].position);
	//glTexCoordPointer(3,GL_FLOAT,sizeof(Vertex),(void *)texCoordOffset);

	//glDrawElements(GL_TRIANGLES,numFaces*3,GL_UNSIGNED_INT,indices);
	glDrawArrays(GL_TRIANGLES,0,numFaces*3);
	glBindBuffer(GL_ARRAY_BUFFER,0);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}


void MeshGraphicsObject::setTransform(float scale,float xTrans/* =0 */, float yTrans/* =0 */, float zTrans/* =0 */, float xRot/* =0 */, float yRot/* =0 */, float zRot/* =0 */)
{
	//don't bother to implement own matrix class yet
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(xTrans,yTrans,zTrans);
	glRotatef(float(xRot*180.0f/PI),1,0,0);
	glRotatef(float(yRot*180.0f/PI),0,1,0);
	glRotatef(float(zRot*180.0f/PI),0,0,1);
	glScalef(scale,scale,scale);
	glGetFloatv(GL_MODELVIEW_MATRIX,transform);
	glPopMatrix();

}

//MeshGraphicsObject::MeshGraphicsObject(const char* fileName, float scale/* =1.0f */, float xTrans/* =0 */, float yTrans/* =0 */, float zTrans/* =0 */)
MeshGraphicsObject::MeshGraphicsObject(const char* fileName, float scale/* =1.0f */, float xTrans/* =0 */, float yTrans/* =0 */, float zTrans/* =0 */, float xRot/* =0 */, float yRot/* =0 */, float zRot/* =0 */)
{
	vertices=NULL;
	numFaces=0;
	
	loadMesh(fileName);
	setTransform(scale,xTrans,yTrans,zTrans,xRot,yRot,zRot);

	//vbo setup
	glGenBuffers(1,&vboRef);
	glBindBuffer(GL_ARRAY_BUFFER,vboRef);
	glBufferData(GL_ARRAY_BUFFER,sizeof(Vertex)*numFaces*3,vertices, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER,0);

	//material
	m_mat=new Material(g_shaderSys); //global bad
	//sett mat fra et annet sted
}


void MeshGraphicsObject::render()
{
	glMultMatrixf(transform);

	/*if(shaderSys)
		shaderSys->enableProfiles();
	if(defaultVertexShader) {
		defaultVertexShader->bind();
		cgGLSetStateMatrixParameter( defaultVertexShader->getParameter("worldViewProj"), CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgGLSetStateMatrixParameter( defaultVertexShader->getParameter("worldView"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY );
		cgUpdateProgramParameters(defaultVertexShader->getProgram());
	}
	if(defaultFragmentShader)
		defaultFragmentShader->bind();
	*/
	if(m_mat) 
		m_mat->bind();

	glEnable(GL_NORMALIZE); //maybe unnecessary
	drawTriangles();
	glDisable(GL_NORMALIZE); //maybe unnecessary

	//if(shaderSys)
	//	shaderSys->disableProfiles();
	if(m_mat)
		m_mat->release();
}


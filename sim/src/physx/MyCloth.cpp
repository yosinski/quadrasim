//from nvidia ex.
//modified!
//does not support tearing anymore

#include <stdio.h>
#include "MyCloth.h"
#include "Stream.h"
#include "wavefront.h"
#include "NxCooking.h"
#include "cooking.h"
#include "../base/kmath.h"


// -----------------------------------------------------------------------
MyCloth::MyCloth(NxScene *scene, NxClothDesc &desc, char *objFileName, NxReal scale, NxVec3* offset, char *texFilename)
  : mVertexRenderBuffer(NULL), mScene(NULL), mCloth(NULL), mClothMesh(NULL),
    mIndexRenderBuffer(NULL), mTempTexCoords(NULL), mNumTempTexCoords(0), mTexId(0)
{
	mInitDone = false;
	NxClothMeshDesc meshDesc;
	generateObjMeshDesc(meshDesc, objFileName, scale, offset, texFilename != NULL);
	init(scene, desc, meshDesc);
	if (texFilename)
		createTexture(texFilename);
}


// -----------------------------------------------------------------------
MyCloth::MyCloth(NxScene *scene, NxClothDesc &desc, NxReal w, NxReal h, NxReal d, char *texFilename)
  : mVertexRenderBuffer(NULL), mScene(NULL), mCloth(NULL), mClothMesh(NULL),
    mIndexRenderBuffer(NULL), mTempTexCoords(NULL), mNumTempTexCoords(0), mTexId(0)
{
	mInitDone = false;
	NxClothMeshDesc meshDesc;
	generateRegularMeshDesc(meshDesc, w, h, d, texFilename != NULL);
	init(scene, desc, meshDesc);
	if (texFilename) 
		createTexture(texFilename);
}

// -----------------------------------------------------------------------
MyCloth::MyCloth(NxScene *scene, NxClothDesc &desc, float height, float radius, int heightDivs, int aroundDivs)
: mVertexRenderBuffer(NULL), mScene(NULL), mCloth(NULL), mClothMesh(NULL),
mIndexRenderBuffer(NULL), mTempTexCoords(NULL), mNumTempTexCoords(0), mTexId(0)
{
	mInitDone = false;
	NxClothMeshDesc meshDesc;
	generateMuscleMeshDesc(meshDesc,height,radius,heightDivs,aroundDivs);
	init(scene, desc, meshDesc);
}


// -----------------------------------------------------------------------
void MyCloth::init(NxScene *scene, NxClothDesc &desc, NxClothMeshDesc &meshDesc)
{
	mScene = scene;
	desc.flags |= NX_CLF_VISUALIZATION; //enable debug rendering

	cookMesh(meshDesc);
	releaseMeshDescBuffers(meshDesc);

	allocateReceiveBuffers(meshDesc.numVertices, meshDesc.numTriangles);
	desc.clothMesh = mClothMesh;
	desc.meshData = mReceiveBuffers;
	mCloth = scene->createCloth(desc);
	mInitDone = true;
}

// -----------------------------------------------------------------------
MyCloth::~MyCloth()
{
	if (mInitDone) {
		if (mCloth) mScene->releaseCloth(*mCloth);
		if (mClothMesh) mScene->getPhysicsSDK().releaseClothMesh(*mClothMesh);
		releaseReceiveBuffers();
		free (mVertexRenderBuffer);
		free (mIndexRenderBuffer);
	}
}

// -----------------------------------------------------------------------
bool MyCloth::generateObjMeshDesc(NxClothMeshDesc &desc, char *filename, NxReal scale, NxVec3* offset, bool textured)
{
	WavefrontObj wo;
	wo.loadObj(filename, textured);
	if (wo.mVertexCount == 0) return false;

	NxVec3 myOffset(0.f);
	if (offset != NULL)
		myOffset = *offset;

	desc.numVertices				= wo.mVertexCount;
	desc.numTriangles				= wo.mTriCount;
	desc.pointStrideBytes			= sizeof(NxVec3);
	desc.triangleStrideBytes		= 3*sizeof(NxU32);
	desc.vertexMassStrideBytes		= sizeof(NxReal);
	desc.vertexFlagStrideBytes		= sizeof(NxU32);
	desc.points						= (NxVec3*)malloc(sizeof(NxVec3)*desc.numVertices);
	desc.triangles					= (NxU32*)malloc(sizeof(NxU32)*desc.numTriangles*3);
	desc.vertexMasses				= 0;
	desc.vertexFlags				= 0;
	desc.flags						= NX_CLOTH_MESH_WELD_VERTICES;
	desc.weldingDistance			= 0.0001f;
	desc.numHierarchyLevels         = 3; //no idea what is best here

	mMaxVertices = wo.mVertexCount;
	mMaxIndices  = 3 * wo.mTriCount;

	// copy positions and indices
	NxVec3 *vSrc = (NxVec3*)wo.mVertices;
	NxVec3 *vDest = (NxVec3*)desc.points;
	for (int i = 0; i < wo.mVertexCount; i++, vDest++, vSrc++) 
		*vDest = (*vSrc)*scale + myOffset;
	memcpy((NxU32*)desc.triangles, wo.mIndices, sizeof(NxU32)*desc.numTriangles*3);

	if (textured)
	{
		mNumTempTexCoords = desc.numVertices;
		mTempTexCoords = (GLfloat *)malloc(sizeof(GLfloat) * mNumTempTexCoords * 2);
		memcpy((NxU32*)mTempTexCoords, wo.mTexCoords, sizeof(GLfloat)*mNumTempTexCoords*2);

		mIndexRenderBuffer = (NxU32*)malloc(sizeof(NxU32) * mMaxIndices);
		memset(mIndexRenderBuffer, 0, sizeof(NxU32) * mMaxIndices);
		for (NxU32 i = 0; i < desc.numTriangles; i++)
		{
			assert((desc.flags & NX_CF_16_BIT_INDICES) == 0);
			NxU32* tri = (NxU32*)(((char*)desc.triangles) + (desc.triangleStrideBytes * i));
			mIndexRenderBuffer[3*i+0] = tri[0];
			mIndexRenderBuffer[3*i+1] = tri[1];
			mIndexRenderBuffer[3*i+2] = tri[2];
		}
	}
	else
	{
		mNumTempTexCoords = 0;
		mTempTexCoords = NULL;
	}

	return true;
}

// -----------------------------------------------------------------------
void MyCloth::generateRegularMeshDesc(NxClothMeshDesc &desc, NxReal w, NxReal h, NxReal d, bool texCoords)
{
	int numX = (int)(w / d) + 1;
	int numY = (int)(h / d) + 1;

	desc.numVertices				= (numX+1) * (numY+1);
	desc.numTriangles				= numX*numY*2;
	desc.pointStrideBytes			= sizeof(NxVec3);
	desc.triangleStrideBytes		= 3*sizeof(NxU32);
	desc.vertexMassStrideBytes		= sizeof(NxReal);
	desc.vertexFlagStrideBytes		= sizeof(NxU32);
	desc.points						= (NxVec3*)malloc(sizeof(NxVec3)*desc.numVertices);
	desc.triangles					= (NxU32*)malloc(sizeof(NxU32)*desc.numTriangles*3);
	desc.vertexMasses				= 0;
	desc.vertexFlags				= 0;
	desc.flags						= 0;

	mMaxVertices = desc.numVertices;
	mMaxIndices  = 3 * desc.numTriangles;

	int i,j;
	NxVec3 *p = (NxVec3*)desc.points;
	for (i = 0; i <= numY; i++) {
		for (j = 0; j <= numX; j++) {
			p->set(d*j, 0.0f, d*i); 
			p++;
		}
	}

	if (texCoords) {
		mTempTexCoords = (GLfloat *)malloc(sizeof(GLfloat)*2*desc.numVertices);
		GLfloat *f = mTempTexCoords;
		GLfloat dx = 1.0f; if (numX > 0) dx /= numX;
		GLfloat dy = 1.0f; if (numY > 0) dy /= numY;
		for (i = 0; i <= numY; i++) {
			for (j = 0; j <= numX; j++) {
				*f++ = j*dx;
				*f++ = i*dy;
			}
		}
		mNumTempTexCoords = desc.numVertices;
	}
	else
	{
		mNumTempTexCoords = 0;
		mTempTexCoords = NULL;
	}

	NxU32 *id = (NxU32*)desc.triangles;
	for (i = 0; i < numY; i++) {
		for (j = 0; j < numX; j++) {
			NxU32 i0 = i * (numX+1) + j;
			NxU32 i1 = i0 + 1;
			NxU32 i2 = i0 + (numX+1);
			NxU32 i3 = i2 + 1;
			if ((j+i)%2) {
				*id++ = i0; *id++ = i2; *id++ = i1;
				*id++ = i1; *id++ = i2; *id++ = i3;
			}
			else {
				*id++ = i0; *id++ = i2; *id++ = i3;
				*id++ = i0; *id++ = i3; *id++ = i1;
			}
		}
	}

}

// -----------------------------------------------------------------------
void MyCloth::generateMuscleMeshDesc(NxClothMeshDesc &desc, float height, float radius, int heightDivs, int aroundDivs)
{
	desc.numVertices				= aroundDivs*(heightDivs+1);
	desc.numTriangles				= aroundDivs*heightDivs*2;
	desc.pointStrideBytes			= sizeof(NxVec3);
	desc.triangleStrideBytes		= 3*sizeof(NxU32);
	desc.vertexMassStrideBytes		= sizeof(NxReal);
	desc.vertexFlagStrideBytes		= sizeof(NxU32);
	desc.points						= (NxVec3*)malloc(sizeof(NxVec3)*desc.numVertices);
	desc.triangles					= (NxU32*)malloc(sizeof(NxU32)*desc.numTriangles*3);
	desc.vertexMasses				= 0;
	desc.vertexFlags				= 0;
	desc.flags						= 0;
	desc.flags						= NX_CLOTH_MESH_WELD_VERTICES;
	desc.weldingDistance			= 0.0001f;

	mMaxVertices = desc.numVertices;
	mMaxIndices  = 3 * desc.numTriangles;

	//points
	NxVec3* p = (NxVec3*)desc.points;
	for(int h=0; h<=heightDivs; h++) {
		for(int i=0; i<aroundDivs; i++) {
			double hDelta=((double)h)/heightDivs;
			double theta=((double)i)/aroundDivs*2.0*PI;
			double r=sin(hDelta*PI)*radius; //find radius of muscle, use sin func for now
			double x=r*cos(theta);
			double y=height*hDelta;
			double z=r*sin(theta);
			p->set((float)x,(float)y,(float)z);
			p++;
		}
	}


	//indices
	NxU32 *ind = (NxU32*)desc.triangles;
	for(int h=0; h<heightDivs; h++) {
		for(int i=0; i<aroundDivs; i++) {
			NxU32 i0 = h*aroundDivs + i;
			NxU32 i1 = h*aroundDivs + ((i+1)%aroundDivs); //wrap around
			NxU32 i2 = (h+1)*aroundDivs + i;
			NxU32 i3 = (h+1)*aroundDivs + ((i+1)%aroundDivs); //wrap around
			*ind++ = i0; *ind++ = i2; *ind++ = i1;
			
			*ind++ = i1; *ind++ = i2; *ind++ = i3;
		}
	}


	//texcoords
	mNumTempTexCoords = 0;
	mTempTexCoords = NULL;
}




// -----------------------------------------------------------------------
void MyCloth::releaseMeshDescBuffers(const NxClothMeshDesc& desc)
{
	NxVec3* p = (NxVec3*)desc.points;
	NxU32* t = (NxU32*)desc.triangles;
	NxReal* m = (NxReal*)desc.vertexMasses;
	NxU32* f = (NxU32*)desc.vertexFlags;
	free(p);
	free(t);
	free(m);
	free(f);
}

// -----------------------------------------------------------------------
bool MyCloth::cookMesh(NxClothMeshDesc& desc)
{
	// Store correct number to detect tearing mesh in time
	mLastNumVertices = desc.numVertices;

	// we cook the mesh on the fly through a memory stream
	MemoryWriteBuffer wb;
	assert(desc.isValid());
	bool success = CookClothMesh(desc, wb);

	if (!success) 
		return false;

	MemoryReadBuffer rb(wb.data);
	mClothMesh = mScene->getPhysicsSDK().createClothMesh(rb);
	return true;
}

// -----------------------------------------------------------------------
void MyCloth::allocateReceiveBuffers(int numVertices, int numTriangles)
{
	// here we setup the buffers through which the SDK returns the dynamic cloth data
	if (mVertexRenderBuffer == NULL) {
		mVertexRenderBuffer = (RenderBufferVertexElement*)malloc(sizeof(RenderBufferVertexElement) * mMaxVertices);
		memset(mVertexRenderBuffer, 0, sizeof(RenderBufferVertexElement) * mMaxVertices);
	}
	if (mIndexRenderBuffer == NULL) {
		mIndexRenderBuffer = (NxU32*)malloc(sizeof(NxU32) * mMaxIndices);
		memset(mIndexRenderBuffer, 0, sizeof(NxU32) * mMaxIndices);
	}

	mReceiveBuffers.verticesPosBegin         = &(mVertexRenderBuffer[0].position.x);
	mReceiveBuffers.verticesNormalBegin      = &(mVertexRenderBuffer[0].normal.x);
	mReceiveBuffers.verticesPosByteStride    = sizeof(RenderBufferVertexElement);
	mReceiveBuffers.verticesNormalByteStride = sizeof(RenderBufferVertexElement);
	mReceiveBuffers.maxVertices              = mMaxVertices;
	mReceiveBuffers.numVerticesPtr           = &mNumVertices;

	NxU32 maxIndices = 3*numTriangles;
	mReceiveBuffers.indicesBegin             = mIndexRenderBuffer;
	mReceiveBuffers.indicesByteStride        = sizeof(NxU32);
	mReceiveBuffers.maxIndices               = maxIndices;
	mReceiveBuffers.numIndicesPtr            = &mNumIndices;

	if (mNumTempTexCoords > 0)
	{
		// Copy Tex Coords from temp buffers to graphics buffer
		assert(mNumTempTexCoords == numVertices);

		for (NxU32 i = 0; i < mNumTempTexCoords; i++)
		{
			mVertexRenderBuffer[i].texCoord[0] = mTempTexCoords[2*i+0];
			mVertexRenderBuffer[i].texCoord[1] = mTempTexCoords[2*i+1];
		}

		// Get rid of temp buffer
		mNumTempTexCoords = 0;
		free (mTempTexCoords);
		mTempTexCoords = NULL;
	}

	// the parent index information would be needed if we used textured cloth
	mReceiveBuffers.parentIndicesBegin       = (NxU32*)malloc(sizeof(NxU32)*mMaxVertices);
	mReceiveBuffers.parentIndicesByteStride  = sizeof(NxU32);
	mReceiveBuffers.maxParentIndices         = mMaxVertices;
	mReceiveBuffers.numParentIndicesPtr      = &mNumParentIndices;

	mReceiveBuffers.dirtyBufferFlagsPtr = &mMeshDirtyFlags;

	mMeshDirtyFlags = 0;
	mNumVertices = 0;
	mNumIndices = 0;
	mNumParentIndices = 0;
}

// -----------------------------------------------------------------------
void MyCloth::releaseReceiveBuffers()
{
	free (mReceiveBuffers.parentIndicesBegin); // Parent Indices is always allocated
	mReceiveBuffers.setToDefault();
}

// -----------------------------------------------------------------------
void MyCloth::draw()
{
	static NxU32 numVertices = mNumVertices;
	NxU32 numElements = mNumIndices;
	numVertices = mNumVertices;

	if (mTexId > 0) 
		updateTextureCoordinates();
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(RenderBufferVertexElement), &(mVertexRenderBuffer[0].position.x));
	glNormalPointer(GL_FLOAT, sizeof(RenderBufferVertexElement), &(mVertexRenderBuffer[0].normal.x));

	if (mTexId) {
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, sizeof(RenderBufferVertexElement), &(mVertexRenderBuffer[0].texCoord[0]));
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, mTexId);
		glColor4f(1.0f, 1.0f, 1.0f,1.0f);
	}

	glDrawElements(GL_TRIANGLES, numElements, GL_UNSIGNED_INT, mIndexRenderBuffer);

	if (mTexId) {
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisable(GL_TEXTURE_2D);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}


// -----------------------------------------------------------------------
bool MyCloth::createTexture(char *filename)
{
/*	BmpLoader bl;
	if (!bl.loadBmp(filename)) return false;

	glGenTextures(1, &mTexId);
	if (!mTexId) return false;
	glBindTexture(GL_TEXTURE_2D, mTexId);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
		bl.mWidth, bl.mHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, bl.mRGB);
*/
	return true;
}

// -----------------------------------------------------------------------
void MyCloth::updateTextureCoordinates()
{
	//is this necessary when not tearing?
	NxU32 numVertices = *mReceiveBuffers.numVerticesPtr;
	NxU32 *parent = (NxU32 *)mReceiveBuffers.parentIndicesBegin + mLastNumVertices;
	for (NxU32 i = mLastNumVertices; i < numVertices; i++, parent++) {
		mVertexRenderBuffer[i].texCoord[0] = mVertexRenderBuffer[*parent].texCoord[0];
		mVertexRenderBuffer[i].texCoord[1] = mVertexRenderBuffer[*parent].texCoord[1];
	}

	mLastNumVertices = numVertices;
}

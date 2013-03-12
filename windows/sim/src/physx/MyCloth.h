//from nvidia example
//modified heavily

#ifndef MYCLOTH_H
#define MYCLOTH_H

#include "../graphics/glstuff.h"
#include "NxPhysics.h"
#include "VertexWelder.h"

class MyCloth 
{
public:
	MyCloth(NxScene *scene, NxClothDesc &desc, char *objFileName, NxReal scale, NxVec3* offset = NULL, char *texFilename = NULL);
	MyCloth(NxScene *scene, NxClothDesc &desc, NxReal w, NxReal h, NxReal d, char *texFilename = NULL);
	MyCloth(NxScene *scene, NxClothDesc &desc, float height, float radius, int heightDivs, int aroundDivs);
	~MyCloth();

	void draw();
	NxCloth *getNxCloth() { return mCloth; }

private:
	void init(NxScene *scene, NxClothDesc &desc, NxClothMeshDesc &meshDesc);
	bool generateObjMeshDesc(NxClothMeshDesc &desc, char *filename, NxReal scale, NxVec3* offset, bool textured);
	void generateRegularMeshDesc(NxClothMeshDesc &desc, NxReal w, NxReal h, NxReal d, bool texCoords);
	void generateMuscleMeshDesc(NxClothMeshDesc &desc, float height, float radius, int heightDivs, int aroundDivs);
	void releaseMeshDescBuffers(const NxClothMeshDesc& desc);
	bool cookMesh(NxClothMeshDesc& desc);
	void allocateReceiveBuffers(int numVertices, int numTriangles);
	void releaseReceiveBuffers();
	bool createTexture(char *filename);
	void updateTextureCoordinates();

	bool mInitDone;
	NxMeshData mReceiveBuffers;
	NxScene *mScene;
	NxCloth *mCloth;
	NxClothMesh *mClothMesh;

	GLuint mTexId;

	struct RenderBufferVertexElement
	{
		NxVec3 position;
		NxVec3 normal;
		float texCoord[2];
	};

	RenderBufferVertexElement* mVertexRenderBuffer;
	NxU32* mIndexRenderBuffer;
	GLfloat *mTempTexCoords;
	NxU32   mNumTempTexCoords;

	NxU32 mMaxVertices;
	NxU32 mMaxIndices;
	NxU32 mNumIndices;
	NxU32 mNumParentIndices;
	NxU32 mNumVertices;
	NxU32 mLastNumVertices;
	NxU32 mMeshDirtyFlags;

	VertexWelder* mVertexWelder;
};

#endif

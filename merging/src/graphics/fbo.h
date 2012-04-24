#ifndef _FBO_H
#define _FBO_H

#include "glstuff.h"

class FrameBufferObject
{
public:
	FrameBufferObject(int width, int height, bool useDepthBuffer);
	~FrameBufferObject();
	void bind();
	void unBind();
	void bindTexture();

	int width,height;
private:
	GLuint texture;
	GLuint fbo;
	GLuint depthbuffer;
};



#endif
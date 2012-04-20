#ifndef _TEXTURE_H
#define _TEXTURE_H

#include "glstuff.h"

class Texture
{
public:
	Texture(const char *filename);
	//also for render target
	void bind();

private:
	GLuint textureID;
};



#endif
#include "../base/system.h"
#include "Texture.h"
#include "glstuff.h"


Texture::Texture(const char *filename)
{
	glGenTextures(1,&textureID); 
	glBindTexture(GL_TEXTURE_2D,textureID); 

	if(GL_FALSE==glfwLoadTexture2D(filename, GLFW_BUILD_MIPMAPS_BIT))
		systemError("could not load tex");
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); 
}
void Texture::bind()
{
	glBindTexture(GL_TEXTURE_2D, textureID);  
}


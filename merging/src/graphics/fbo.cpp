#include "fbo.h"
#include "../base/system.h"
#include "glstuff.h"

FrameBufferObject::FrameBufferObject(int width, int height, bool useDepthBuffer)
{
	if(!glewIsSupported("GL_EXT_framebuffer_object")) 
		systemError("fbo not supported");

	this->width=width;
	this->height=height;

	glGenFramebuffersEXT(1, &fbo);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);


	depthbuffer=0;
	if(useDepthBuffer) {
		glGenRenderbuffersEXT(1, &depthbuffer);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, width, height);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer);
		/*GLuint depthTex;
		glGenTextures(1, &depthTex);
		glBindTexture(GL_TEXTURE_2D, depthTex);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);	
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8_EXT, width, height, 0, GL_DEPTH_STENCIL_EXT, GL_UNSIGNED_INT_24_8_EXT, NULL);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,GL_TEXTURE_2D, depthTex, 0);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,GL_STENCIL_ATTACHMENT_EXT,GL_TEXTURE_2D, depthTex, 0);
		*/
	}

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //necessary on nvidia!
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //necessary on nvidia!
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_CLAMP);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,  width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, texture, 0);
	
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	if(GL_FRAMEBUFFER_COMPLETE_EXT!=status)
		systemError("framebuffer object not complete");

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
}


void FrameBufferObject::bind()
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,fbo);
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0,0,width, height);
}

void FrameBufferObject::unBind()
{
	glPopAttrib();
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
}


void FrameBufferObject::bindTexture()
{
	glBindTexture(GL_TEXTURE_2D,texture);
}

FrameBufferObject::~FrameBufferObject()
{
	glDeleteFramebuffersEXT(1,&fbo);
	glDeleteTextures(1,&texture);
	if(depthbuffer)
		glDeleteRenderbuffersEXT(1, &depthbuffer);
}
#define GLFW_DLL

#include "../base/system.h"
#include "../base/tools.h"
#include "../base/kmath.h"
#include "Texture.h"
#include "fbo.h"
#include "shaders.h"
#include "GraphicsObject.h"
#include <stdio.h>
#include <math.h>
#include "graphics.h"
#include "glstuff.h"
#include <stdlib.h>
#include "../physics.h"
#include "jpeglib.h"
#include "Overlay.h"

void drawScene(bool follow=false,bool drawGround=true);

//globally accessible for now
bool g_visualize=true;
bool g_enableGlow=false;
bool g_enableOverlay=false;
bool g_enableNoise=false;

ShaderSystem* g_shaderSys=NULL;


Shader* g_defaultVertexShader=NULL;
Shader* g_defaultFragmentShader=NULL;
Shader* texFragmentShader=NULL;

Shader *passThruVertexShader=NULL;
Shader *blurFragmentShader=NULL;

Texture *tempTex=NULL;
Texture *tempTex2=NULL;

FrameBufferObject *fbo;
#define GLOW_PASSES 4
FrameBufferObject *lowResBuffers[GLOW_PASSES];
FrameBufferObject *blurredBuffers[GLOW_PASSES];

//for camera
Vec3 followTarget;
float followPanAmount;
NxMat34 firstPersonMat;
int camType=CAM_MOUSEPAN;

//for shaders:
Vec3 g_lightPos;
float g_viewMat[16];

//overlays
Overlay *g_overlay=NULL;
NoiseOverlay *g_noiseOverlay=NULL;


void terminateGraphics()
{
	if(fbo)
		delete fbo;

	/*//incompatible syntax: for each
	for each(Shader* s in g_shaders)
		delete s;
	g_shaders.clear();*/
}

void toggleRetrace()
{
	static int retrace=0;
	retrace=!retrace;
	glfwSwapInterval(retrace);
}

void initRendering()
{
	//glew - gl extensions
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		printf("Error: %s\n", glewGetErrorString(err));
		systemError("glew init error");
	}
	//glfwSwapInterval( 1 ); //TEST - vsync enable - 

	//geometries
	//SphereGraphicsObject::initVertices(3);
	SphereGraphicsObject::initVertices(2);
	CapsuleGraphicsObject::initVertices(24);

	//could make some storage for the textures later
	tempTex2=new Texture("data/concfloor1024.tga");
	tempTex=new Texture("data/Threadplate1024_bright.tga");
	//tempTex=new Texture("data/concfloor512.tga");

	//shader stuff
	g_shaderSys=new ShaderSystem();
	g_defaultVertexShader=new Shader(g_shaderSys,"data/shaders/v_test.cg","v_test",Shader::VERTEX);
	//g_shaders.push_back(g_defaultVertexShader);
	g_defaultFragmentShader=new Shader(g_shaderSys,"data/shaders/f_notex.cg","f_notex",Shader::FRAGMENT);
	//g_shaders.push_back(g_defaultFragmentShader);
	texFragmentShader=new Shader(g_shaderSys,"data/shaders/f_test.cg","f_test",Shader::FRAGMENT);

	passThruVertexShader=new Shader(g_shaderSys,"data/shaders/v_passthru.cg","v_test",Shader::VERTEX);
	blurFragmentShader=new Shader(g_shaderSys,"data/shaders/f_blur.cg","f_blur",Shader::FRAGMENT);

	//fbo
	int fboWidth,fboHeight;
	//glfwGetWindowSize( &fboWidth, &fboHeight );
	//fboWidth=256;
	//fboHeight=256;
	fboWidth=128;
	fboHeight=128;
	fbo = new FrameBufferObject(fboWidth,fboHeight,true);

	for(int i=0;i<GLOW_PASSES;i++) {
		fboWidth/=2;
		fboHeight/=2;
		lowResBuffers[i] = new FrameBufferObject(fboWidth,fboHeight,false);
		blurredBuffers[i] = new FrameBufferObject(fboWidth,fboHeight,false);
	}


	setFollowTargetAndPanning(Vec3(0,5,0),0.0f);
	firstPersonMat.id();
	camType=CAM_MOUSEPAN;

	//global light pos
	g_lightPos.set(100,100,250);

	//overlays
	g_overlay=new Overlay;
	g_overlay->addOverlay("default","data/overlays/default.tga");
	g_noiseOverlay=new NoiseOverlay();
	g_noiseOverlay->addOverlay("dummy","data/overlays/noise1.tga");

	//opengl states
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_FLAT);
}



void glow(FrameBufferObject *sourceBuffer, float glowAmount)
{
	//downsample buffers
	FrameBufferObject *prev=sourceBuffer;
	glEnable(GL_TEXTURE_2D);
	//downsample
	for(int i=0;i<GLOW_PASSES;i++) {
		lowResBuffers[i]->bind();
		prev->bindTexture();
		drawScreenAlignedQuad();
		lowResBuffers[i]->unBind(); //could optimize away
	}

	//blur buffers
	g_shaderSys->enableProfiles();
	passThruVertexShader->bind();
	blurFragmentShader->bind();
	for(int i=0;i<GLOW_PASSES;i++) {
		blurredBuffers[i]->bind();
		lowResBuffers[i]->bindTexture();

		float tStep=1.0f/lowResBuffers[i]->width;
		cgGLSetParameter1f(blurFragmentShader->getParameter("tStep"), tStep);
		cgUpdateProgramParameters(blurFragmentShader->getProgram());

		drawScreenAlignedQuad();
		blurredBuffers[i]->unBind(); //could optimize away
	}
	g_shaderSys->disableProfiles();

	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA,GL_ONE); //blir det feil? er alle src pixler alpha 1? må revisere blendregler (src alpha fra tex? hva slags tex?)
	glBlendFunc(GL_ONE,GL_ONE);
	glEnable(GL_TEXTURE_2D);

	float amount=1.0f/GLOW_PASSES*glowAmount;
	for(int i=0;i<GLOW_PASSES;i++) {
		blurredBuffers[i]->bindTexture();
		//glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		//glColor4f(1,1,1,amount); //hvis bruker alpha
		glColor4f(amount,amount,amount,1);
		drawScreenAlignedQuad();
	}

	//glEnable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glColor4f(1,1,1,1);
	glDisable(GL_TEXTURE_2D);
}



void drawGroundPlane(float size)
{
	tempTex->bind();
	if(g_shaderSys) 
		g_shaderSys->enableProfiles();
	g_defaultVertexShader->bind();
	texFragmentShader->bind();

	//set a parameter
	cgGLSetParameter4f(g_defaultVertexShader->getParameter("lightPos"), g_lightPos.x,g_lightPos.y,g_lightPos.z,1); //move to bind place
	cgGLSetMatrixParameterfc(g_defaultVertexShader->getParameter("view"),g_viewMat);
	cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldViewProj"), CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY );
	cgGLSetStateMatrixParameter( g_defaultVertexShader->getParameter("worldView"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY );
	cgUpdateProgramParameters(g_defaultVertexShader->getProgram());

	float s=size/2;
	//float t=80.0f; //texture scale
	//float t=size*1.0f; //texture scale
	float t=size*0.05f; //texture scale
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	
	glMatrixMode(GL_MODELVIEW);
	glBegin(GL_QUADS);
	glNormal3f(0,1,0);
	glTexCoord2f(0,0);
	glVertex3f(-s,0,+s);
	glTexCoord2f(t,0);
	glVertex3f(+s,0,+s);
	glTexCoord2f(t,t);
	glVertex3f(+s,0,-s);
	glTexCoord2f(0,t);
	glVertex3f(-s,0,-s);
	glEnd();

	if(g_shaderSys) 
		g_shaderSys->disableProfiles();

	glMatrixMode(GL_MODELVIEW);
	glDisable(GL_TEXTURE_2D);
	//glEnable(GL_LIGHTING);
}


void drawScreenAlignedQuad()
{
	drawScreenAlignedQuad(0,0,1,1);
}

void drawScreenAlignedQuad(float u0,float v0,float u1,float v1)
{
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION); 
	glPushMatrix(); 
	glLoadIdentity(); 
	glBegin(GL_QUADS); 
	//glColor3f(1,1,1);
	glTexCoord2f(u0,v0);
	glVertex3i(-1, -1, -1); 
	glTexCoord2f(u1,v0);
	glVertex3i(1, -1, -1); 
	glTexCoord2f(u1,v1);
	glVertex3i(1, 1, -1); 
	glTexCoord2f(u0,v1);
	glVertex3i(-1, 1, -1); 
	glEnd(); 
	glPopMatrix(); 
	glMatrixMode(GL_MODELVIEW); 
	glPopMatrix();

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
}

//simple shadows:
/*if (shadows) {
	const static float ShadowMat[]={ 1,0,0,0, 0,0,0,0, 0,0,1,0, 0,0,0,1 };
	glPushMatrix();
	glMultMatrixf(ShadowMat);
	glDisable(GL_LIGHTING);
	glColor4f(0.05f, 0.1f, 0.15f,1.0f);
	glDrawElements(GL_TRIANGLES, numElements, GL_UNSIGNED_INT, mIndexRenderBuffer);
	glColor4f(1.0f, 1.0f, 1.0f,1.0f);
	glEnable(GL_LIGHTING);
	glPopMatrix();
}*/

void mouseCam()
{
	//inputs, should be separate
	int x,y;
	glfwGetMousePos( &x, &y );
	int wheel=glfwGetMouseWheel();
	int width,height;
	glfwGetWindowSize( &width, &height );

	float distance=140.0f-wheel*5.0f;
	//float distance=10.0f-wheel*0.2f;
	float camX= sin( ((float)x)/width*PI*3.0f ) * distance;
	float camY=-(((float)y)/height-0.5f)*distance*6;
	float camZ= cos( ((float)x)/width*PI*3.0f ) * distance;
	float targetY= 0.0f;
	
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt( camX, camY, camZ, 0.0f, targetY, 0.0f, 0.0f, 1.0f, 0.0f );  //eye, target, up
	firstPersonMat.t.set(camX,camY,camZ);
}

#define TARGET_SMOOTH_SIZE 20
Vec3 targetBuf[TARGET_SMOOTH_SIZE];
int bufOffs=0;

void setFollowTargetAndPanning(Vec3 target,float panAmount)
{
	//followTarget=target;
	followPanAmount=panAmount;

	targetBuf[bufOffs]=target;
	bufOffs++;
	bufOffs%=TARGET_SMOOTH_SIZE;
	Vec3 avg;
	for(int i=0;i<TARGET_SMOOTH_SIZE;i++) {
		avg+=targetBuf[i];
	}
	avg*=(1.0f/TARGET_SMOOTH_SIZE);
	followTarget=avg;
}

void followCam()
{
	//follow a point
	static unsigned frames=0;
	//float dist=100.0f;
	float dist=50.0f;
	Vec3 pos;
	Vec3 target=followTarget;
	target.y-=5.0f;

	int buttonState=glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT);

	//pan with the mouse if button pressed, otherwise timed panning
	if(GLFW_PRESS==buttonState) {
		int x,y;
		glfwGetMousePos( &x, &y );
		int wheel=glfwGetMouseWheel();
		int width,height;
		glfwGetWindowSize( &width, &height );

		dist=dist-wheel*2.0f;
		pos.x=target.x+sin( ((float)x)/width*PI*3.0f ) * dist;
		pos.y=target.y-(((float)y)/height-0.5f)*dist*6;
		pos.z=target.z+cos( ((float)x)/width*PI*3.0f ) * dist;
	}
	else {
		float angle=followPanAmount;	
		frames++;
		pos.x=target.x+dist*sin(angle);
		//pos.y=target.y+0.5f*dist+0.5f*dist*cos(angle/2.0f);
		//pos.y=target.y+0.3f*dist+0.5f*dist*cos(angle/2.0f);
		//pos.y=0.3f*dist+0.5f*dist*cos(angle/2.0f);
		pos.y=0.1f*dist+0.5f*dist*cos(angle/2.0f);
		//pos.y=0.5f*dist+0.5f*dist*cos(angle/2.0f);
		pos.z=target.z+dist*cos(angle);
	}

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	//gluLookAt(pos.x,pos.y,pos.z,target.x,target.y,target.z,0.0f,1.0f,0.0f ); //eye, target, up
	gluLookAt(pos.x,pos.y,pos.z,target.x,target.y/2.0,target.z,0.0f,1.0f,0.0f ); //eye, target, up
	firstPersonMat.t.set(pos.x,pos.y,pos.z);

}


void firstPersonCam()
{
	static NxVec3 rotVector(0,0,0);
	static double lastTime = glfwGetTime();
	double currentTime=glfwGetTime();
	double deltaTime=currentTime-lastTime; //for framerate insensitivity
	lastTime=currentTime;
	
	float rotFactor=float(2.5f*deltaTime);
	//NxVec3 rotVector(0,0,0);
	if(glfwGetKey(GLFW_KEY_LEFT)) 
		rotVector+=NxVec3(0,1,0)*rotFactor;
	if(glfwGetKey(GLFW_KEY_RIGHT)) 
		rotVector+=NxVec3(0,-1,0)*rotFactor;
	if(glfwGetKey(GLFW_KEY_UP)) 
		rotVector+=NxVec3(-1,0,0)*rotFactor;
	if(glfwGetKey(GLFW_KEY_DOWN)) 
		rotVector+=NxVec3(1,0,0)*rotFactor;
	NxMat33 xRot;
	xRot.rotX(rotVector.x);
	NxMat33 yRot;
	yRot.rotY(rotVector.y);
	NxMat33 orientation;
	orientation.multiply(yRot,xRot); //new orientation around global axes

	double advanceFactor=30.0*deltaTime;
	NxVec3 advanceVector(0,0,0);
	if(glfwGetKey('W')) 
		advanceVector+=NxVec3(0,0,-1);
	if(glfwGetKey('X')) 
		advanceVector+=NxVec3(0,0,1);
	if(glfwGetKey('A')) 
		advanceVector+=NxVec3(-1,0,0);
	if(glfwGetKey('D')) 
		advanceVector+=NxVec3(1,0,0);
	if(glfwGetKey('E')) 
		advanceVector+=NxVec3(0,1,0);
	if(glfwGetKey('C')) 
		advanceVector+=NxVec3(0,-1,0);
	advanceVector*=float(advanceFactor);

	NxVec3 globalPos;
	firstPersonMat.multiply(advanceVector,globalPos);
	firstPersonMat.t=globalPos;
	firstPersonMat.M=orientation; //want to always rotate around world axes

	NxVec3 localLookAt(0,0,-1); //local look at direction
	NxVec3 localUp(0,1,0); //local up vector
	NxVec3 globalLookAt;
	NxVec3 globalUp;

	firstPersonMat.multiply(localLookAt,globalLookAt);
	firstPersonMat.M.multiply(localUp,globalUp);

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt(globalPos.x,globalPos.y,globalPos.z,globalLookAt.x,globalLookAt.y,globalLookAt.z,globalUp.x,globalUp.y,globalUp.z);
}



void drawScene(bool follow,bool drawGround)
{
	int width,height;
	glfwGetWindowSize( &width, &height );

	//DRAWING STUFF
	glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
	glClear( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT);

	//projection setup
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( 65.0f, (GLfloat)width/(GLfloat)height, 1.0f, 10000.0f );
	//gluPerspective( 65.0f, (GLfloat)width/(GLfloat)height, 1.0f, 100.0f );

	//init modelview and set cam
	if(CAM_FOLLOW==camType)
		followCam();
	else if(CAM_FIRSTPERSON==camType)
		firstPersonCam();
	else {
		mouseCam(); //maybe remove later if not needed anymore
	}
	
	glGetFloatv(GL_MODELVIEW_MATRIX,g_viewMat); //save view (camera) matrix for later use in shaders

	//draw ground
	if(drawGround)
		drawGroundPlane(1500);
	//draw objects
	for(unsigned i=0;i<parts.size();i++) {
		parts[i]->render();
	}

	//include in parts?
	glEnable(GL_LIGHTING);
	glShadeModel(GL_SMOOTH);
	for(unsigned i=0;i<cloths.size();i++) {
		cloths[i]->draw();
	}
}


void updateGraphics(void)
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
	drawScene();
	if(g_enableGlow) {
		fbo->bind();
		drawScene(false,false);
		fbo->unBind();
		glow(fbo,0.90f);
	}
	if(g_enableOverlay && g_overlay)
		g_overlay->draw();
	if(g_enableNoise && g_noiseOverlay)
		g_noiseOverlay->draw();
}



void renderDebugInfo()
{
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
	gDebugRenderer.renderData(*gScene->getDebugRenderable());
	glEnable(GL_DEPTH_TEST);
}



//code by Adam Chou
bool screenshot(unsigned int width, unsigned int height, char *path, int quality)
{
	bool ret=false;

	struct jpeg_compress_struct cinfo; // the JPEG OBJECT
	struct jpeg_error_mgr jerr; // error handler struct
	unsigned char *row_pointer[1]; // pointer to JSAMPLE row[s]
	GLubyte *pixels=0, *flip=0;
	FILE *shot;
	int row_stride; // width of row in image buffer

	if((shot=fopen(path, "wb"))!=NULL) { // jpeg file
		// initializatoin
		cinfo.err = jpeg_std_error(&jerr); // error handler
		jpeg_create_compress(&cinfo); // compression object
		jpeg_stdio_dest(&cinfo, shot); // tie stdio object to JPEG object
		row_stride = width * 3;

		pixels = (GLubyte *)malloc(sizeof(GLubyte)*width*height*3);
		flip = (GLubyte *)malloc(sizeof(GLubyte)*width*height*3);

		if (pixels!=NULL && flip!=NULL) {
			// save the screen shot into the buffer
			//glReadBuffer(GL_FRONT_LEFT);
			glPixelStorei(GL_PACK_ALIGNMENT, 1);
			glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

			// give some specifications about the image to save to libjpeg
			cinfo.image_width = width;
			cinfo.image_height = height;
			cinfo.input_components = 3; // 3 for R, G, B
			cinfo.in_color_space = JCS_RGB; // type of image 

			jpeg_set_defaults(&cinfo);
			jpeg_set_quality(&cinfo, quality, TRUE);
			jpeg_start_compress(&cinfo, TRUE);

			// OpenGL writes from bottom to top.
			// libjpeg goes from top to bottom.
			// flip lines.
			for(unsigned y=0;y<height;y++) {
				for(unsigned x=0;x<width;x++) {                
					flip[(y*width+x)*3] = pixels[((height-1-y)*width+x)*3];
					flip[(y*width+x)*3+1] = pixels[((height-1-y)*width+x)*3+1];
					flip[(y*width+x)*3+2] = pixels[((height-1-y)*width+x)*3+2];
				}
			}

			// write the lines
			while (cinfo.next_scanline < cinfo.image_height) {
				row_pointer[0] = &flip[cinfo.next_scanline * row_stride];
				jpeg_write_scanlines(&cinfo, row_pointer, 1);
			}
			ret=true;
			// finish up and free resources
			jpeg_finish_compress(&cinfo);
			jpeg_destroy_compress(&cinfo);
		}
		fclose(shot);
	}

	if(pixels!=0)
		free(pixels);
	if(flip!=0)
		free(flip);
	return ret;
}



ImageRecorder::ImageRecorder(int frameSkip,char* destPathAndBaseFileName,int quality)
{
	this->frameSkip=frameSkip;
	this->quality=quality;
	strcpy(this->destPath,destPathAndBaseFileName);
	lastRecordedFrame=0;
	recordFrameCounter=0;
	recordFileNumber=0;
	active=false;
}


void ImageRecorder::start() {
	active=true;
	printf("image recording activated\n");
}


void ImageRecorder::update()
{
	if(!active)
		return;

	if(recordFrameCounter-lastRecordedFrame > frameSkip) {
		int width,height;
		glfwGetWindowSize( &width, &height );
		char fullFileName[200];
		sprintf(fullFileName,"%s%05d.jpg",destPath,recordFileNumber);
		screenshot(width,height,fullFileName,quality);
		lastRecordedFrame=recordFrameCounter;
		recordFileNumber++;
	}
	recordFrameCounter++;
}


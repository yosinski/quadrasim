#ifndef _GRAPHICS_H
#define _GRAPHICS_H


#include <vector>
#include "../base/Vec3.h"
#include "shaders.h"
#include "glstuff.h"
#include "Texture.h"
#include "fbo.h"

void updateGraphics(void);
void renderDebugInfo();
void drawGroundPlane(float size);
void initRendering();
void terminateGraphics();
void drawScreenAlignedQuad();
void setFollowTargetAndPanning(Vec3 target,float panAmount=0.0f);
void glow(FrameBufferObject *sourceBuffer, float glowAmount);
void toggleRetrace();

//bool screenshot(unsigned int width, unsigned int height, char *path, int quality); //borrowed func

extern Texture *tempTex;
extern Texture *tempTex2;

extern ShaderSystem* g_shaderSys;
extern Shader* g_defaultVertexShader;
extern Shader* g_defaultFragmentShader;
//extern std::vector<Shader*> g_shaders;

//for shaders
extern Vec3 g_lightPos;
extern float g_viewMat[16];


extern FrameBufferObject *fbo;

enum CameraType{CAM_MOUSEPAN,CAM_FOLLOW,CAM_FIRSTPERSON};
extern int camType;

extern bool g_visualize;

#define ENABLE_POSTPROCESSING 1
extern bool g_enableGlow;



struct ImageRecorder {
	bool active;
	int frameSkip;
	int quality;
	char destPath[200];
	int lastRecordedFrame;
	int recordFrameCounter;
	int recordFileNumber;

	//ImageRecorder(int frameSkip,char* destPathAndBaseFileName);
	ImageRecorder(int frameSkip,char* destPathAndBaseFileName,int quality=100);
	void start();
	void update();

};


#endif
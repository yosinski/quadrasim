#ifndef _SYSTEM_H
#define _SYSTEM_H
/*
#ifndef GLFW_DLL
#define GLFW_DLL
#endif

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h> //removes the warnings
*/

#include "../graphics/glstuff.h"

double updateFPS();
//void initWindow();
void initWindow(int xres=640,int yres=480,bool fullscreen=false,bool multisample=false);
void updateWindowSize();
void systemError(const char *msg);
//void processGlobalKeys();
void GLFWCALL processGlobalKeys( int key=0, int action=0 );

#endif
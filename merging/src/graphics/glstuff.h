#ifndef _GLSTUFF_H
#define _GLSTUFF_H
//common include file, trying to avoid gl include order probs

#ifndef GLFW_DLL
#define GLFW_DLL
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif
//#define WIN32_LEAN_AND_MEAN
//#include <windows.h> //removes the warnings


#include <GL/glew.h>
#include <GL/glfw.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#endif

#include "../base/system.h"
#include "shaders.h"
#include "glstuff.h"
#include <Cg/cg.h>
#include <Cg/cgGL.h>
#include <stdio.h>

ShaderSystem::ShaderSystem()
{
	cgContext=cgCreateContext();
	checkError("creating context");
	cgGLSetDebugMode(CG_FALSE);
	cgSetParameterSettingMode(cgContext,CG_DEFERRED_PARAMETER_SETTING);

	//profiles
	vertexProfile=cgGLGetLatestProfile(CG_GL_VERTEX);
	cgGLSetOptimalOptions(vertexProfile);
	checkError("selecting vertex profile");

	fragmentProfile=cgGLGetLatestProfile(CG_GL_FRAGMENT);
	cgGLSetOptimalOptions(fragmentProfile);
	checkError("selecting fragment profile");

	printf("vertex profile:   %s\n",
		cgGetProfileString(cgGLGetLatestProfile(CG_GL_VERTEX)));
	printf("geometry profile: %s\n",
		cgGetProfileString(cgGLGetLatestProfile(CG_GL_GEOMETRY)));
	printf("fragment profile: %s\n",
		cgGetProfileString(cgGLGetLatestProfile(CG_GL_FRAGMENT)));



}

void ShaderSystem::checkError(const char *situation)
{
	CGerror error;
	const char *string = cgGetLastErrorString(&error);
	if (error != CG_NO_ERROR) {
		printf("%s: %s\n",situation, string);
		if (error == CG_COMPILER_ERROR) {
			printf("%s\n", cgGetLastListing(cgContext));
		}
		systemError("cg error");
	}
}

void ShaderSystem::enableProfiles()
{
	cgGLEnableProfile(fragmentProfile);
	checkError("enabling fragment profile");
	cgGLEnableProfile(vertexProfile);
	checkError("enabling vertex profile");
}

void ShaderSystem::disableProfiles()
{
	cgGLDisableProfile(vertexProfile);
	checkError("disabling vertex profile");
	cgGLDisableProfile(fragmentProfile);
	checkError("disabling fragment profile");
}



//////////////////////////////////////////////////////////////////////////
// SHADER
//////////////////////////////////////////////////////////////////////////

CGprogram Shader::getProgram()
{
	return program;
}

Shader::Shader(ShaderSystem *shaderContext,const char *filename, const char *entryPoint,int shaderType)
{
	CGprofile profile;
	if(NULL==shaderContext)
		systemError("invalid shader context");
	context=shaderContext;
	if(VERTEX==shaderType) {
		profile=shaderContext->vertexProfile;
	}
	else if(FRAGMENT==shaderType) {
		profile=shaderContext->fragmentProfile;
	}
	else
		systemError("invalid shader type");

	program=cgCreateProgramFromFile(shaderContext->cgContext,CG_SOURCE,filename,profile,entryPoint,NULL);
	context->checkError("creating shader program from file");
	cgGLLoadProgram(program);
	context->checkError("loading shader program");
}

void Shader::bind()
{
	cgGLBindProgram(program);
	context->checkError("binding vertex program");
}


CGparameter Shader::getParameter(const char *name)
{
	CGparameter parameter;
	parameter=cgGetNamedParameter(program,name);
#ifdef _DEBUG
	context->checkError("could not get parameter"); 
#endif
	return parameter;
}


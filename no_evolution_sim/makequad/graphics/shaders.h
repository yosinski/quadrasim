#ifndef _SHADERS_H
#define _SHADERS_H

#include <Cg/cg.h>

//if one needs to extend to "effects" later, use CGeffect (.cgfx)

class ShaderSystem
{
public:
	void checkError(const char *situation); //checks if an error has occured
	void enableProfiles();
	void disableProfiles();
	ShaderSystem();
//private:
	CGcontext   cgContext;
	CGprofile   vertexProfile;
	CGprofile   fragmentProfile;
};

class Shader
{
public:
	enum ShaderType{VERTEX,FRAGMENT};

	Shader(ShaderSystem *shaderContext,const char *filename, const char *entryPoint,int shaderType);
	void bind();
	CGprogram getProgram(); //this is not so good, should avoid exposing to cg program
	CGparameter getParameter(const char *name);
protected:
	CGprogram program;
	ShaderSystem* context;

};


#endif

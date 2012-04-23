#include "glstuff.h"
#include "../base/system.h"
#include "material.h"
#include "graphics.h"
#include <assert.h>

Shader* Material::m_vertexShader=NULL;
Shader* Material::m_fragmentShader=NULL;


Material::Material(ShaderSystem* shaderSys)
{
	//if(shaderSys==NULL) 
	//	systemError("material: invalid shader system ref");
	assert(shaderSys && "material: invalid shader system ref");
	m_shaderSys=shaderSys;

	if(m_vertexShader==NULL) {
		m_vertexShader=new Shader(shaderSys,"data/shaders/v_stdmaterial.cg","v_stdmaterial",Shader::VERTEX);
		//g_shaders.push_back(m_vertexShader);
	}
	if(m_fragmentShader==NULL) {
		m_fragmentShader=new Shader(shaderSys,"data/shaders/f_stdmaterial.cg","f_stdmaterial",Shader::FRAGMENT);
		//g_shaders.push_back(m_fragmentShader);
	}


	m_color.set(0,0,1);
}


void Material::bind()
{
	//bytt til egen shader
	if(m_shaderSys)
		m_shaderSys->enableProfiles();
	
	if(m_vertexShader) {
		m_vertexShader->bind();
		cgGLSetStateMatrixParameter(m_vertexShader->getParameter("worldViewProj"), CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY);
		cgGLSetStateMatrixParameter(m_vertexShader->getParameter("worldView"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY);
		cgGLSetParameter4f(m_vertexShader->getParameter("lightPos"), g_lightPos.x,g_lightPos.y,g_lightPos.z,1);
		cgGLSetMatrixParameterfc(m_vertexShader->getParameter("view"),g_viewMat);
		cgUpdateProgramParameters(m_vertexShader->getProgram());
	}
	if(m_fragmentShader) {
		m_fragmentShader->bind();
		cgGLSetParameter3f(m_fragmentShader->getParameter("col"),m_color.x,m_color.y,m_color.z);
		cgUpdateProgramParameters(m_fragmentShader->getProgram());
	}
}


void Material::release()
{
	if(m_shaderSys)
		m_shaderSys->disableProfiles();

}

void Material::setColor(float r,float g,float b)
{

	m_color.set(r,g,b);
}
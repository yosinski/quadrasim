#ifndef _MATERIAL_H
#define _MATERIAL_H

#include "shaders.h"
#include "../base/Vec3.h"

class Material
{
public:
	Material(ShaderSystem* shaderSys);
	virtual void bind();
	virtual void release();
	void setColor(float r,float g,float b);

private:
	ShaderSystem* m_shaderSys;
	static Shader* m_vertexShader;
	static Shader* m_fragmentShader;
	Vec3 m_color;

};

#endif
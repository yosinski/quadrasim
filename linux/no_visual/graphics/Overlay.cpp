#include "Overlay.h"

#include "graphics.h"
#include "Texture.h"
#include "../base/system.h"
#include "../base/tools.h"

void Overlay::addOverlay(const char *refname, const char *filename)
{
	Texture* tex=new Texture(filename);
	if(!tex)
		systemError("could not load overlay texture");
	overlays[std::string(refname)]=tex;

}

void Overlay::draw()
{
	Texture *tex;
	tex=overlays["default"]; //hack


	//g_shaderSys->enableProfiles();
	//passThruVertexShader->bind();
	////blurFragmentShader->bind(); 
	glEnable(GL_TEXTURE_2D);
	tex->bind();
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_ONE,GL_ONE);
	glEnable(GL_TEXTURE_2D);
	glColor4f(1,1,1,1);
	drawScreenAlignedQuad();
	glDisable(GL_BLEND);
	glDisable(GL_TEXTURE_2D);
	//g_shaderSys->disableProfiles();
}


/************************************************************************/
/* NOISEOVERLAY                                                         */
/************************************************************************/


NoiseOverlay::NoiseOverlay()
{
	currentOverlay=0;
	blendAmount=0.25f;
	//create noise later here?
}

void NoiseOverlay::draw()
{
	currentOverlay++;
	float u,v;
	u=randRangef(0.2f,1);
	v=randRangef(0.2f,1);

	Texture *tex;
	//tex=overlays["default"]; //hack
	std::map<std::string, Texture*>::iterator it=overlays.begin();
	tex=it->second;

	glEnable(GL_TEXTURE_2D);
	tex->bind();
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	//hva slags blend?
	//glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_ZERO,GL_ONE_MINUS_SRC_COLOR);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	glEnable(GL_TEXTURE_2D);
	glColor4f(1,1,1,blendAmount);
	drawScreenAlignedQuad(0,0,u,v);
	glDisable(GL_BLEND);
	glDisable(GL_TEXTURE_2D);
	//g_shaderSys->disableProfiles();


}


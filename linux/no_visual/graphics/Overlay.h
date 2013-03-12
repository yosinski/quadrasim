#ifndef _OVERLAY_H
#define _OVERLAY_H


#include <string>
#include <map>
#include "Texture.h"

struct Overlay 
{
public:
	virtual void draw();
	void addOverlay(const char *refname, const char *filename);

protected:
	std::map<std::string, Texture*> overlays;

};


struct NoiseOverlay : public Overlay
{
public:
	NoiseOverlay();
	virtual void draw();
	float blendAmount;
protected:
	int currentOverlay;
};

#endif

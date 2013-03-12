#ifndef _VERTEX_H
#define _VERTEX_H

#include "../base/Vec3.h"

class Vertex {
public:
	Vec3 position;
	Vec3 texCoord;
	Vec3 normal;

	Vertex(void);
};



#endif
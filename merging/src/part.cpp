#include "part.h"
#include "graphics/GraphicsObject.h"
#include "graphics/glstuff.h"
#include "NxPhysics.h"
#include "physics.h"
#include <assert.h>


Part::Part(void)
{
	act=NULL;
}

Part::Part(NxActor *physicsEquivalent)
{
	assert(physicsEquivalent && "part: physics equivalent invalid");
	this->act=physicsEquivalent;
}

void Part::render()
{
	assert(act && "partrender: physics equivalent invalid");
	glPushMatrix();
	float glMat[16];
	NxActor* actor;
	actor=act;
	if(NULL==actor) return;
	actor->getGlobalPose().getColumnMajor44(glMat);
	glMultMatrixf(glMat);

	if(actor->userData!=NULL && gRenderUserData) {
		glPushMatrix();
		((GraphicsObject*)actor->userData)->render();
		glPopMatrix();
	}
	else {
		//render according to shape descriptions in actor
		for(unsigned s=0; s < actor->getNbShapes(); s++) {
			NxShape *shape = actor->getShapes()[s];

			glPushMatrix();
			shape->getLocalPose().getColumnMajor44(glMat);
			glMultMatrixf(glMat);

			//render shapes
			if(shape->getFlag(NX_TRIGGER_ENABLE)) {
				//do nothing, triggers are not to be rendered and have different userdata
			}
			else if(shape->userData!=NULL) {
				((GraphicsObject*)shape->userData)->render();
			}
			else {
				NxShapeType type=shape->getType();
				if(NX_SHAPE_BOX==type) {
					NxBoxShape *sh=(NxBoxShape *)shape;
					NxVec3 dim=sh->getDimensions();
					BoxGraphicsObject g(dim.x,dim.z,dim.y);  //wrond dimensions
					g.render();
				}
				else if(NX_SHAPE_CAPSULE==type) {
					NxCapsuleShape *sh=(NxCapsuleShape *)shape;
					float radius=sh->getRadius();
					float height=sh->getHeight();
					CapsuleGraphicsObject g(radius,height);
					g.render();
				}
				else if(NX_SHAPE_SPHERE==type) {
					NxSphereShape *sh=(NxSphereShape *)shape;
					float radius=sh->getRadius();
					SphereGraphicsObject g(radius);
					g.render();
				}
				else {
					//render a default sphere if shape type unknown
					SphereGraphicsObject sg(1);
					sg.render();
				}

			}
			glPopMatrix();
		}
	}
	glPopMatrix();
}

void Part::addTouchSensor(NxVec3 localPos,bool* statusMeter,float radius)
{
	assert(act && "add touch sensor: physics equivalent invalid");
	NxSphereShapeDesc trig;
	trig.localPose.t=localPos;
	trig.radius=radius;
	trig.shapeFlags |= NX_TRIGGER_ENABLE;
	trig.userData=statusMeter;
	NxShape* trigShape=act->createShape(trig);
	assert(trigShape && "add touch sensor: could not create shape");
}	

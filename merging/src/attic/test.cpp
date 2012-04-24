#include "system.h"
#include "graphics.h"
#include "physics.h"
#include "tools.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include <GL/glfw.h>
#include "NxPhysics.h"


struct Part
{
	//palBody *physicsEquivalent;
	NxActor *physicsEquivalent;
	int type;
	float size; //simple now
};

std::vector<Part> parts;

NxActor* createSphere(const Point3D& pos, float size=1)
{
	if(gScene == NULL || !gScene->isWritable()) return NULL;	

	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius		= size;
	sphereDesc.localPose.t	= NxVec3(0, 0, 0);

	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 10.0f;
	actorDesc.globalPose.t	= NxVec3(pos.x,pos.y,pos.z);	
	return gScene->createActor(actorDesc);
	//could link userData to the Part 
}

NxActor* createCube(const Point3D& pos, float size=2)
{
	if(gScene == NULL || !gScene->isWritable()) return NULL;	
	NxBodyDesc bodyDesc;
	bodyDesc.angularDamping	= 0.5f;
	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions = NxVec3((float)size, (float)size, (float)size);
	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&boxDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 10.0f;
	actorDesc.globalPose.t  = NxVec3(pos.x,pos.y,pos.z);
	//gScene->createActor(actorDesc)->userData = (void*)size_t(size);
	return gScene->createActor(actorDesc);
}


NxActor* createCapsule(const Point3D& pos, float size=1)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;

	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.radius		= 1.0f;
	capsuleDesc.height		= size;
	capsuleDesc.localPose.t = NxVec3(0, 0, 0);

	actorDesc.shapes.pushBack(&capsuleDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 10.0f;
	actorDesc.globalPose.t	= NxVec3(pos.x,pos.y,pos.z);

	return gScene->createActor(actorDesc);
}





void dropObjects(int type)
{
	const int numBalls=2;
	float dist=12.0;
	for(int i=0;i<numBalls;i++) {
		float size=randRangef(0.2,2);
		if(0==type) {
			Point3D pos(randRangef(-1,1)*dist,40.0f, randRangef(-1,1)*dist);
			Part part;
			part.physicsEquivalent=createCube(pos,size);
			if(part.physicsEquivalent==NULL) return;
			part.size=size;
			part.type=0;
			parts.push_back(part);
		}
		else if(1==type) {
			Point3D pos(randRangef(-1,1)*dist,40.0f, randRangef(-1,1)*dist);
			Part part;
			part.physicsEquivalent=createSphere(pos,size);
			if(part.physicsEquivalent==NULL) return;
			part.size=size;
			part.type=1;
			parts.push_back(part);
		}
		else if(2==type) {
			Point3D pos(randRangef(-1,1)*dist,40.0f, randRangef(-1,1)*dist);
			Part part;
			part.physicsEquivalent=createCapsule(pos,size);
			if(part.physicsEquivalent==NULL) return;
			part.size=size;
			part.type=2;
			parts.push_back(part);
		}
	}
}


NxRevoluteJoint* createRevoluteJoint(NxActor* a0, NxActor* a1, NxVec3 globalAnchor, NxVec3 globalAxis)
{
	NxRevoluteJointDesc revDesc;

	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	revDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	revDesc.projectionMode = NX_JPM_POINT_MINDIST;
	revDesc.projectionDistance = 1.0f;
	revDesc.projectionAngle = 0.0872f;	//about 5 degrees in radians.

	return (NxRevoluteJoint*)gScene->createJoint(revDesc);
}

//palRevoluteLink *link;
NxRevoluteJoint *testJoint;
void dropLinkedObjects()
{
	float size=2;
	Point3D pos1(-2,2,-2);
	Part part1;
	part1.physicsEquivalent=createCube(pos1,size);
	if(part1.physicsEquivalent==NULL) return;
	part1.size=size;
	part1.type=0;
	parts.push_back(part1);

	Point3D pos2(+2,2,2);
	Part part2;
	part2.physicsEquivalent=createCube(pos2,size);
	if(part2.physicsEquivalent==NULL) return;
	part2.size=size;
	part2.type=0;
	parts.push_back(part2);

	testJoint=createRevoluteJoint(part1.physicsEquivalent,part2.physicsEquivalent,NxVec3(0,2,0),NxVec3(0,1,0));

}


int main(void)
{
	srand(3213214);
	initPhysics();
	initWindow();
	initRendering();
	
/*	dropObjects(0);
	dropObjects(1);
	dropObjects(2);*/
	dropLinkedObjects();

	static int frameCount=0;

	// Main loop
	bool running = true;
	while( running )
	{
		updateFPS();
		updateWindowSize();

		int x,y;
		glfwGetMousePos( &x, &y );
		int wheel=glfwGetMouseWheel();
		int width,height;
		glfwGetWindowSize( &width, &height );

		if(glfwGetKey( GLFW_KEY_SPACE)) dropObjects(0);
		if(glfwGetKey( GLFW_KEY_ENTER)) dropObjects(1);
		if(glfwGetKey( GLFW_KEY_LSHIFT)) dropObjects(2);

		//PHYSICS STUFF
		NxMotorDesc mDesc;
		mDesc.maxForce=2000;
		float mSpeed;
		if(sin(frameCount/20.0)>0) mSpeed=-3;
		else mSpeed=3;
		mDesc.velTarget=mSpeed;
		testJoint->setMotor(mDesc);

		gScene->simulate(1.0f/60.0f);

		frameCount++;

		//DRAWING STUFF
		glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
		glClear( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT);

		float distance=40.0f-wheel*2.5f;
		float camX= sin( ((float)x)/width*PI*2.0f ) * distance;
		float camY=(((float)y)/height-0.5f)*distance*2;
		float camZ= cos( ((float)x)/width*PI*2.0f ) * distance;
		float targetY= 0.0f;

		//projection setup
		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();
		gluPerspective( 65.0f, (GLfloat)width/(GLfloat)height, 1.0f,4*distance );

		//modelview - camera
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		gluLookAt( camX, camY, camZ, 0.0f, targetY, 0.0f, 0.0f, 1.0f, 0.0f );  //eye, target, up

		//draw ground
		drawGroundPlane(250);
		//draw objects

		for(unsigned  i=0;i<parts.size();i++) {
			glPushMatrix();
			float glMat[16];
			parts[i].physicsEquivalent->getGlobalPose().getColumnMajor44(glMat);
			glMultMatrixf(glMat);

			if(parts[i].type==0) 
				drawBox(parts[i].size);
			if(parts[i].type==1)
				drawSphere(parts[i].size);
			if(parts[i].type==2) 
					drawCapsule(1,parts[i].size);
			
			glPopMatrix();
		}

		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		glfwSwapBuffers(); //screen update
		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}

	glfwTerminate();
	terminatePhysics();
}


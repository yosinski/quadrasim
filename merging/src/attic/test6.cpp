//test app.
#include "system.h"
#include "kmath.h"
#include "graphics.h"
#include "GraphicsObject.h"
#include "MeshGraphicsObject.h"
#include "physics.h"
#include "tools.h"
#include "part.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include <GL/glfw.h>
#include <Cg/cgGL.h>
#include "NxPhysics.h"

//GA stuff
#include <ga/GASimpleGA.h>
#include <ga/GABin2DecGenome.h>
#include <ga/std_stream.h>
#define cout STD_COUT

float fitness(GAGenome &);	//fitness function


std::vector<Part *> parts;
std::vector<NxRevoluteJoint *> joints; //kan evt. bruke egen joint-klasse etterhvert

void updatePhysics();
void updateGraphics();
void renderDebugInfo();

void createSnake()
{
	float length=2.0;
	Part *lastPart;

	float x,y,z;
	x=y=z=0;
	
	//length of one capsule = length + 2 * rad(=1)
	Part *p0=new Part();
	p0->physicsEquivalent=createCapsule(Point3D(0,0,0),length,false); 
	p0->size=length;
	CapsuleGraphicsObject *cg0=new CapsuleGraphicsObject(1,length);  //this creates a lot of duplicates, doesn't matter at the moment
	p0->physicsEquivalent->getShapes()[0]->userData=cg0;
	parts.push_back(p0);
	lastPart=p0;

	for(int seg=1;seg<10;seg++) {
		y=(length+2)*seg;
		Part *p=new Part();
		p->size=length;
		p->physicsEquivalent=createCapsule(Point3D(0,y,0),length,false);
		CapsuleGraphicsObject *cg=new CapsuleGraphicsObject(1,length);  //this creates a lot of duplicates, doesn't matter at the moment
		p->physicsEquivalent->getShapes()[0]->userData=cg;
		parts.push_back(p);

		NxRevoluteJoint *j=createRevoluteJoint(lastPart->physicsEquivalent,p->physicsEquivalent,NxVec3(x,y,z),NxVec3(0,0,1));
		if(j==NULL) systemError("funka ikke");
		joints.push_back(j);
		lastPart=p;

	}
}

MeshGraphicsObject *robMesh;
NxActor* createRobotPart( const NxMat34 &transform, float size)
{
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	float rad=1.0f;

	NxCapsuleShapeDesc cd;
	cd.radius		= rad;
	cd.height		= size;
	CapsuleGraphicsObject *cg=new CapsuleGraphicsObject(rad,size);  //this creates a lot of duplicates, doesn't matter at the moment
	cd.userData=cg;
	cd.localPose.t = NxVec3(0, 0, 0);	//capsule is on "floor"
	actorDesc.shapes.pushBack(&cd);

	NxCapsuleShapeDesc cd2;
	cd2.radius		= rad;
	cd2.height		= size;
	CapsuleGraphicsObject *cg2=new CapsuleGraphicsObject(rad,size);  //this creates a lot of duplicates, doesn't matter at the moment
	cd2.userData=cg2;
	cd2.localPose.t = cd.localPose.t+NxVec3(2*rad, 0, 0);	
	actorDesc.shapes.pushBack(&cd2);

	//perpendicular
	NxCapsuleShapeDesc cd3;
	cd3.radius		= rad;
	cd3.height		= size;
	CapsuleGraphicsObject *cg3=new CapsuleGraphicsObject(cd3.radius,cd3.height);  //this creates a lot of duplicates, doesn't matter at the moment
	cd3.userData=cg3;
	cd3.localPose.t = cd2.localPose.t+NxVec3(size/2, -size/2+0.3f*size, 0);	//capsule is on "floor"
	cd3.localPose.M.id();
	cd3.localPose.M.rotZ(-PI/2.0); //in radians, not angles like doc says
	actorDesc.shapes.pushBack(&cd3);


	actorDesc.body=&bodyDesc;
	actorDesc.density=10.0f;
	actorDesc.globalPose=transform;

	if(NULL==robMesh) {
		robMesh=new MeshGraphicsObject("data/untitled2.obj");
	}

	//scale down object and orient it according to the shapes.. could have done it the other way round also
	glPushMatrix();
	glLoadIdentity();
	float scale=0.026f;
	glTranslatef(5.2f,-2.4f,0); 
	glRotatef(90,1,0,0);
	glRotatef(90,0,1,0);
	glScalef(scale,scale,scale);
	glGetFloatv(GL_MODELVIEW_MATRIX,robMesh->transform);
	glPopMatrix();


	actorDesc.userData=robMesh;
	return gScene->createActor(actorDesc);
}



void createRobot()
{
	float length=4.0;
	float rad=1.0f;
	Part *lastPart;

	//float x,y,z;
	//x=y=z=0;
	NxMat34 tfm0;
	tfm0.id();
	tfm0.M.rotX(-PI/2);
	tfm0.t.set(0,20,0);
	Part *p0=new Part();
	p0->physicsEquivalent=createRobotPart(tfm0,length); 
	p0->size=length;
	parts.push_back(p0);
	lastPart=p0;

	for(int seg=1;seg<5;seg++) {

		//somewhat hairy local transformation (relative to previous part)
		NxMat34 localTransform;
		localTransform.id();
		localTransform.t.set(2*rad+length+length*0.05f,-length/2+0.3f*length-2*rad,0); //disse verdiene m� bare justeres senere
		NxMat33 xrot,zrot;
		xrot.rotX(PI);
		zrot.rotZ(-PI/2);
		localTransform.M.multiply(zrot,xrot);

		//get world transformation by multiplying with global pose from previous part
		NxMat34 globalTransform=lastPart->physicsEquivalent->getGlobalPose();
		globalTransform.multiply(globalTransform,localTransform);

		//link anchor point
		NxVec3 localLinkPos;
		localLinkPos.set(localTransform.t);
		NxVec3 globalLinkPos=lastPart->physicsEquivalent->getGlobalPose()*localLinkPos;

		//link rotation axis
		NxVec3 localLinkAxis(1,0,0);
		NxVec3 globalLinkAxis;
		lastPart->physicsEquivalent->getGlobalPose().M.multiply(localLinkAxis,globalLinkAxis);

		Part *p=new Part();
		p->size=length;
		p->physicsEquivalent=createRobotPart(globalTransform,length);
		parts.push_back(p);

		NxRevoluteJoint *j=createRevoluteJoint(lastPart->physicsEquivalent,p->physicsEquivalent,globalLinkPos,globalLinkAxis);
		if(j==NULL) systemError("funka ikke");
		joints.push_back(j);
		lastPart=p;

	}
}


void dropObjects(int type)
{
	static double lastTime = 0;
	double currentTime=glfwGetTime();
	double deltaTime=currentTime-lastTime;

	const int numBalls=1;
	float dist=12.0;
	for(int i=0;i<numBalls;i++) {
		float size=randRangef(0.2f,2.0f);
		if(0==type) {
			Point3D pos(randRangef(-1,1)*dist,40.0f, randRangef(-1,1)*dist);
			float width=size;
			float height=randRangef(0.2f,2.0f);
			float depth=randRangef(0.2f,2.0f);
			NxActor *physicsEquivalent=createCube(pos,width,depth,height);  
			if(physicsEquivalent==NULL) return;
			//add a graphics object to it
			BoxGraphicsObject *bg=new BoxGraphicsObject(width,depth,height);
			physicsEquivalent->getShapes()[0]->userData=bg;

			Part *part=new Part(physicsEquivalent,size);
			parts.push_back(part);
		}
		else if(1==type) {
			Point3D pos(randRangef(-1,1)*dist,40.0f, randRangef(-1,1)*dist);
			NxActor *physicsEquivalent=createSphere(pos,size);
			if(physicsEquivalent==NULL) return;
			//add a graphics object to it
			SphereGraphicsObject *sg=new SphereGraphicsObject(size);
			physicsEquivalent->getShapes()[0]->userData=sg;

			Part *part=new Part(physicsEquivalent,size);
			parts.push_back(part);
		}
		else if(2==type) {
			Point3D pos(randRangef(-1,1)*dist,40.0f, randRangef(-1,1)*dist);
			float rad=randRangef(0.5f,2.0f);
			NxActor *physicsEquivalent=createCapsule(pos,size*2,true,rad);
			if(physicsEquivalent==NULL) return;

			//add a graphics object to it
			CapsuleGraphicsObject *cg=new CapsuleGraphicsObject(rad,size*2);
			physicsEquivalent->getShapes()[0]->userData=cg;

			Part *part=new Part(physicsEquivalent,size);
			parts.push_back(part);
		}
		else if(4==type) {
			if(deltaTime>0.1)
				createSnake();
		}
		else if(5==type) {
			if(deltaTime>0.1)
				createRobot();
		}
	}
	lastTime=currentTime;
}




int main(void)
{
	srand(3213214);
	initPhysics();
	initWindow();
	initRendering();

	//HACK HACK GA TEST
	GARandomSeed(1337);

	//default parameters
	GAParameterList params;
	GASimpleGA::registerDefaultParameters(params);
	params.set(gaNnGenerations, 100);
	params.set(gaNpopulationSize, 30);
	params.set(gaNpCrossover, 0.9);
	params.set(gaNpMutation, 0.001);
	params.set(gaNscoreFilename, "bog.dat");
	params.set(gaNscoreFrequency, 10);
	params.set(gaNflushFrequency, 50);

	//read parameters from file, overwrite default settings!
	char *paramFile="settings.txt";
	params.read(paramFile); 

	//target data for fitness function
	unsigned int n=7;
	float *target = new float[n];
	float min[] = {0, 0,   3, -5, 100,    0.001, 0};
	float max[] = {1, 100, 3, -2, 10000, 0.010, 7};
	for(int i=0; i<n; i++)
		target[i] = GARandomFloat(min[i], max[i]);

	//debug printout
	for(int i=0; i<n; i++)
		cout << target[i] << " ";
	cout << "\n";

	//mapping of bits to decimals
	GABin2DecPhenotype map;
	for(int i=0; i<n; i++)
		map.add(8, min[i], max[i]); //8b precision (maybe too low)

	//genome
	GABin2DecGenome genome(map, fitness, (void *)target);

	//start ga and evolve
	GASimpleGA ga(genome);
	ga.parameters(params);
	ga.evolve(1337);

	/*//"manual" evolution
	ga.initialize();
	while(!ga.done()){
	++ga;
	}
	ga.flushScores();*/


	cout << "The GA found:\n" << ga.statistics().bestIndividual() << "\n" << "fitness: " << ga.statistics().bestIndividual().score() << "\n";




	//glfwSwapInterval( 1 ); //TEST - vsync enable - 
	//dropObjects(3);
	//createSnake();
	//createRobot();

	// Main loop
	bool running = true;
	while( running )
	{
		updateFPS();
		updateWindowSize();

		if(glfwGetKey( GLFW_KEY_SPACE)) dropObjects(0);
		if(glfwGetKey( GLFW_KEY_ENTER)) dropObjects(1);
		if(glfwGetKey( GLFW_KEY_LSHIFT)) dropObjects(2);
		if(glfwGetKey( GLFW_KEY_RCTRL)) dropObjects(3);
		if(glfwGetKey( GLFW_KEY_RSHIFT)) dropObjects(4);
		if(glfwGetKey( GLFW_KEY_TAB)) dropObjects(5);

		updatePhysics();
		
		if( !glfwGetKey(GLFW_KEY_F1) ) 
			updateGraphics();

		//m� finne ut etterhvert hva som kan v�re hvor
		gScene->flushStream();
		gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

		if(glfwGetKey( GLFW_KEY_LCTRL)) 
			renderDebugInfo();

		glfwSwapBuffers(); //screen update
		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}

	glfwTerminate();
	terminatePhysics();
}




void renderDebugInfo()
{
	glDisable(GL_DEPTH_TEST);		
	gDebugRenderer.renderData(*gScene->getDebugRenderable());
	glEnable(GL_DEPTH_TEST);

}

void updatePhysics()
{
	static int frameCount=0; //hacky hack
	frameCount++; //hacky hack

	//spring could be an alternative as a kind of servo motor
	for(unsigned j=0;j<joints.size();j++) {
		NxMotorDesc mDesc;
		mDesc.maxForce=1000;
		float mSpeed;
		if(sin((frameCount+j*10)/10.0)>0) mSpeed=-3;
		else mSpeed=3;
		mDesc.velTarget=mSpeed;
		joints[j]->setMotor(mDesc);
	}

	//main simulation step
	gScene->simulate(1.0f/60.0f);
}

void updateGraphics()
{
	int x,y;
	glfwGetMousePos( &x, &y );
	int wheel=glfwGetMouseWheel();
	int width,height;
	glfwGetWindowSize( &width, &height );


	//DRAWING STUFF
	glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
	glClear( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT);

	float distance=40.0f-wheel*2.5f;
	float camX= sin( ((float)x)/width*PI*2.0f ) * distance;
	float camY=-(((float)y)/height-0.5f)*distance*4;
	float camZ= cos( ((float)x)/width*PI*2.0f ) * distance;
	float targetY= 0.0f;


	//projection setup
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( 65.0f, (GLfloat)width/(GLfloat)height, 0.1f,4*distance );

	//modelview - camera
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt( camX, camY, camZ, 0.0f, targetY, 0.0f, 0.0f, 1.0f, 0.0f );  //eye, target, up

	//for shader:
	cgGLSetParameter4f(defaultVertexShader->getParameter("lightPos"), 50,20,50,1);
	cgGLSetStateMatrixParameter( defaultVertexShader->getParameter("view"), CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_IDENTITY );
	cgUpdateProgramParameters(defaultVertexShader->getProgram());



	//draw ground
	drawGroundPlane(250);

	//draw objects
	for(unsigned  i=0;i<parts.size();i++) {
		parts[i]->render();
	}
	
}


float fitness(GAGenome& g)
{
	GABin2DecGenome & genome = (GABin2DecGenome &)g;
	float *sequence = (float *)g.userData();

	float value=genome.nPhenotypes();
	for(int i=0; i<genome.nPhenotypes(); i++)
		value += 1.0 / (1.0 + fabs(genome.phenotype(i) - sequence[i]));
	return value;
}
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
#include "NxPhysics.h"

std::vector<Part *> parts;
std::vector<NxRevoluteJoint *> joints; //kan evt. bruke egen joint-klasse etterhvert

void updatePhysics();
void updateGraphics();

#define DEBUG_PRINT 0

#define MAX_ELEMENTS 1000000
char generationA[MAX_ELEMENTS];
char generationB[MAX_ELEMENTS];


char *rules[256];
unsigned char ruleRanges[256];


struct LState
{
	//transformation matrix - use built-in opengl for proto
	float xRot,yRot,zRot;
	float forwardStep;
	float upStep;
	float thickness;
	int numIterations;
	float randRange;

	LState() {
		xRot=yRot=zRot=45.0f;
		forwardStep=1.0;
		upStep=0.8;
		thickness=1.0;
		numIterations=5;
		randRange=45;
	}

	void printSettings() {
		printf("xRot=%f;\nyRot=%f;\nzRot=%f;\n",xRot,yRot,zRot);
		printf("forwardStep=%f;\nthickness=%f;\n",forwardStep,thickness);
		printf("numIterations=%d;\n",numIterations);
		printf("\n");
	}

};

LState state;


void initLSystem()
{
	//memset(generationA,0,sizeof(generationA));
	//memset(generationB,0,sizeof(generationB));

	memset(rules,0,sizeof(rules));
	memset(ruleRanges,255,sizeof(ruleRanges));


	//alle regler startes nå med 'S'

	//litt kul, prøv forskjellige vinkler  (thomas fant ut at den var veldig kul!)
/*	rules['A']="ypFypFypFypF";
	rules['B']="A[BD[[AACrp]C]";
	rules['C']="BBpCy BBD";
	rules['D']="YpYpFYpFYpF";
	rules['S']="B";*/

	//bygningskomplekser
	/*rules['A']="FyFFFYA";   //opptegning av arm
	rules['B']="[A]uB";            //etasjer
	rules['C']="[RE][B][yuC]";
	ruleRanges['C']=3;
	rules['D']="<*1.2>F<=2>f<=4>FD"; //opptegning av arm
	rules['E']="[D]uE";            //etasjer
	rules['S']="C";
	state.yRot=40;
	state.zRot=60;
	state.xRot=90;
	state.thickness=0.2f;
	state.forwardStep=4.0f;
	state.randRange=20;*/


	//også kul, prøv forskjellige vinkler
	/*rules['A']="ypFypFypFypF";
	rules['B']="A[BD]AACrpC";
	rules['C']="BBpCBBDFFFF";
	rules['D']="PFPFPFPFPFPFPFPF";
	rules['S']="B";
	state.xRot=34.400162;
	state.yRot=45.000000;
	state.zRot=9.499950;
	state.forwardStep=1.459999;
	state.thickness=1.000000;
	state.numIterations=8;*/
	


	//kul med tannhjul
	rules['A']="ypFypFypFypF";
	rules['B']="A[BD]AACrpC";
	rules['C']="BBpCBBDFFFF";
	rules['D']="PFPFPFrD";
	rules['S']="B";
	state.xRot=34.400162;
	state.yRot=45.000000;
	state.zRot=9.499950;
	state.forwardStep=1.459999;
	//state.thickness=1.000000;
	state.thickness=0.7f;
	state.numIterations=8;

	/*rules['A']="ypFypFypFypF";
	rules['B']="A[BD]AACrpC";
	rules['C']="BBpCBBDFFFF";
	rules['D']="PFPFPFrD";
	rules['S']="B";
	state.xRot=34.400162;
	state.yRot=45.000000;
	state.zRot=9.499950;
	state.forwardStep=1.459999;
	state.thickness=1.000000;
	state.numIterations=8;*/



	/*//kul og
	rules['A']="ypFypFypFypF";
	rules['B']="A[[BD]AAC]";
	rules['C']="BpCB[uyD]FF";
	rules['D']="PFPFPFD";
	rules['S']="B";
	//generationA[0]='B';
	state.xRot=52.699883;
	state.yRot=60.099770;
	state.zRot=2.699950;
	state.forwardStep=1.430000;
	state.numIterations=9;*/
	


	//koch
	/*rules['F']="FYFyFyFYF";
	rules['S']="F";
	state.yRot=90;*/

	//tre
	/*rules['X']="Fy[[X]YX]YF[YFX]yX";
	rules['F']="FF";
	rules['S']="X";
	state.xRot=25;
	state.yRot=25;
	state.zRot=25;*/
}

char *curGen=NULL;
void developRules(int numIterations)
{
	memset(generationA,0,sizeof(generationA));
	memset(generationB,0,sizeof(generationB));
	generationA[0]='S';
	curGen=generationA;
	char *nextGen=generationB;

	//debug
	if(DEBUG_PRINT) {
		for(int r=0;r<256;r++) {
			if(rules[r])
				printf("rule %c: %s\n",r,rules[r]);
		}
		printf("\n");
	}

	for(int iteration=0;iteration<numIterations;iteration++) {
		int curPos=0;
		int nextPos=0;

		if(DEBUG_PRINT) printf("cur gen %s\n",curGen);
		
		while(curGen[curPos]!=0) {
			char curToken=curGen[curPos];
			if(rules[curToken]!=NULL && ruleRanges[curToken]>iteration) { //there's a rule for the token
				int rulePos=0;
				while(rules[curToken][rulePos]!='\0') {
					nextGen[nextPos++]=rules[curToken][rulePos++];
					if(nextPos>=MAX_ELEMENTS) systemError("exceeded max string length");;
				}
			}
			else { //just copy the token
				nextGen[nextPos++]=curGen[curPos];
				if(nextPos>=MAX_ELEMENTS) systemError("exceeded max string length");;
			}
			curPos++;
			if(curPos>=MAX_ELEMENTS) systemError("exceeded max string length");;
		}
		if(DEBUG_PRINT) printf("next gen %s\n\n",nextGen);


		char *temp=curGen;
		curGen=nextGen;
		nextGen=temp;
	}
}


struct Operation
{
	char operation;
	float number;
	bool isUsed;
};


void modify(float& val,Operation& op)
{
	if(op.operation=='*')
		val*=op.number;
	else if(op.operation=='+')
		val+=op.number;
	else if(op.operation=='-')
		val-=op.number;
	else if(op.operation=='=')
		val=op.number;

	op.isUsed=true;
}

void renderLString()
{
	if(curGen==NULL) return;
	srand(1337);

	LState pushedState=state;

	BoxGraphicsObject b(2,1,state.thickness);
	//BoxGraphicsObject b(state.forwardStep*state.forwardStep,2,state.thickness);
	SphereGraphicsObject s(1);

	int numPushes=0;
	int maxPushes=8;

	Operation op;
	op.isUsed=true;

	int pos=0;
	while(curGen[pos]!=0) {
		char token=curGen[pos];
		float amount=1.0f;

		switch(token) {
			case 'Y' : //yaw+
				glRotatef(state.yRot,0,1,0);
				break;
			case 'y' : //yaw-
				glRotatef(-state.yRot,0,1,0);
				break;
			case 'P' : //pitch+
				glRotatef(state.zRot,0,0,1);
				break;
			case 'p' : //pitch-
				glRotatef(-state.zRot,0,0,1);
				break;
			case 'R' : //roll+
				glRotatef(state.xRot,1,0,0);
				break;
			case 'r' : //roll-
				glRotatef(-state.xRot,1,0,0);
				break;
			case '[' : //save position
				if(numPushes<maxPushes) {
					glPushMatrix();
					numPushes++;
				}
				break;
			case ']' : //revert to last saved position
				if(numPushes>0) {
					glPopMatrix();
					numPushes--;
				}
				break;
			case '<' : //    <+2.31443>
				{
					int numberPos=0;
					while(curGen[pos+2+numberPos++] != '>');
					char str[20];
					memcpy(str,&curGen[pos+2],numberPos+1);
					str[numberPos]='\0';
					amount=atof(str);
					//printf("operator: %c  number: %f\n",curGen[pos+1],amount);
					op.operation=curGen[pos+1];
					pos+=1+numberPos;
					op.number=amount;
					op.isUsed=false;
				}
				break;
			case 'F' : //forward
				b.render();
				if(!op.isUsed) modify(state.forwardStep,op);
				glTranslatef(state.forwardStep,0,0);
				break;
			case 'f' : //forward no draw
				glTranslatef(state.forwardStep,0,0);
				break;
			case 'u' : //up no draw
				glTranslatef(0,state.upStep,0);
				break;
			case 'x' : //random turn
				glRotatef(randRangef(-state.randRange,state.randRange),1,0,0);
				glRotatef(randRangef(-state.randRange,state.randRange),0,1,0);
				glRotatef(randRangef(-state.randRange,state.randRange),0,0,1);
				break;
		}
		pos++;
	}

	//clear pushed matrix stack 
	while(numPushes>0) {
		glPopMatrix();
		numPushes--;
	}

	state=pushedState;
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

	float distance=100.0f-wheel*2.5f;
	float camX= sin( ((float)x)/width*PI*2.0f ) * distance;
	float camY=-(((float)y)/height-0.5f)*distance*4;
	float camZ= cos( ((float)x)/width*PI*2.0f ) * distance;
	float targetY= 0.0f;

	//projection setup
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( 65.0f, (GLfloat)width/(GLfloat)height, 1.0f,10*distance );

	//modelview - camera
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt( camX, camY, camZ, 0.0f, targetY, 0.0f, 0.0f, 1.0f, 0.0f );  //eye, target, up

	//draw ground
	//drawGroundPlane(250);

	/*//draw objects
	for(unsigned  i=0;i<parts.size();i++) {
		parts[i]->render();
	}*/


	/*
	//testing
	BoxGraphicsObject b(1,2,3);
	glRotatef(45,0,0,1);
	glTranslatef(10,2,0);
	b.render();
	glTranslatef(10,0,0);
	b.render();
	glRotatef(-45,0,0,1);
	glTranslatef(10,0,0);
	b.render();*/

	renderLString();




	//må finne ut etterhvert hva som kan være hvor
	gScene->flushStream();
	gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);

	/*if(glfwGetKey( GLFW_KEY_LCTRL)) {
		glDisable(GL_DEPTH_TEST);		
		gDebugRenderer.renderData(*gScene->getDebugRenderable());
		glEnable(GL_DEPTH_TEST);
	}*/

	glfwSwapBuffers(); //screen update
}





int main(void)
{
	srand(3213214);
	initPhysics();
	initWindow();
	initRendering();

	//glfwSwapInterval( 1 ); //TEST - vsync enable - 

	initLSystem();
	developRules(state.numIterations);

	// Main loop
	bool running = true;
	while( running )
	{
		updateFPS();
		updateWindowSize();

		static double lastTime = 0;
		double currentTime=glfwGetTime();
		double deltaTime=currentTime-lastTime;

		if(glfwGetKey(GLFW_KEY_UP)) state.zRot+=0.1f; 
		if(glfwGetKey(GLFW_KEY_DOWN)) state.zRot-=0.1f;
		if(glfwGetKey(GLFW_KEY_LEFT)) state.yRot+=0.1f;
		if(glfwGetKey(GLFW_KEY_RIGHT)) state.yRot-=0.1f;
		if(glfwGetKey(GLFW_KEY_LCTRL)) state.xRot+=0.1f;
		if(glfwGetKey(GLFW_KEY_RCTRL)) state.xRot-=0.1f;
		if(glfwGetKey(GLFW_KEY_LALT)) state.forwardStep+=0.01f;
		if(glfwGetKey(GLFW_KEY_RALT)) { 
			state.forwardStep-=0.01f;
			if(state.forwardStep<0.01f) state.forwardStep=0.01f;
		}
		if(deltaTime>0.1) {
			if(glfwGetKey(GLFW_KEY_PAGEDOWN)) { 
				state.numIterations-=1;
				if(state.numIterations==0) state.numIterations=1;
				//initLSystem();
				//resetLSystemDevelopment();
				developRules(state.numIterations);
			}
			if(glfwGetKey(GLFW_KEY_PAGEUP)) { 
				state.numIterations+=1;
				if(state.numIterations==20) state.numIterations=19;
				//initLSystem();
				//resetLSystemDevelopment();
				developRules(state.numIterations);
			}
			if(glfwGetKey(GLFW_KEY_TAB)) state.printSettings();
			lastTime=currentTime;
		}


		updatePhysics();
		updateGraphics();
 
		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}

	glfwTerminate();
	terminatePhysics();
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


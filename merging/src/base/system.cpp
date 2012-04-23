#include "system.h"
#include <stdio.h>
#include <stdlib.h>
#include "../graphics/glstuff.h"
#include "../graphics/graphics.h"
#include "../physics.h"

//must be called once per frame!
double updateFPS()
{
	static double fps=0;
	static int deltaFrames=0;
	static double lastTime = glfwGetTime();

	double currentTime=glfwGetTime();
	// Calculate and display FPS (frames per second)
	if( (currentTime-lastTime) > 1.0 || deltaFrames == 0 )
	{
		fps = (double)deltaFrames / (currentTime-lastTime);
		char    titlestr[ 200 ];
		sprintf( titlestr, "test (%.1f fps)", fps);
		glfwSetWindowTitle( titlestr );
		lastTime = currentTime;
		deltaFrames = 0;
	}
	deltaFrames++;
	return fps;
}

void initWindow(int xres,int yres,bool fullscreen,bool multisample)
{
	glfwInit();

	if(multisample)
		glfwOpenWindowHint(GLFW_FSAA_SAMPLES,4);
	int mode=GLFW_WINDOW;
	if(fullscreen)
		mode=GLFW_FULLSCREEN;
	if( !glfwOpenWindow( xres, yres, 0,0,0,8, 24,0, mode ) ) {
		glfwTerminate();
		systemError("could not open window");
	}
	//glfwEnable( GLFW_STICKY_KEYS );
	glfwSwapInterval( 0 ); //vsync enable
	glfwSetKeyCallback( processGlobalKeys );
}

void systemError( const char *msg )
{
	printf("Error: %s\n",msg);
	glfwTerminate();
	//physx cleanup here - or call physics genereal cleanup func
	exit(1);
}

void updateWindowSize()
{
	int width,height;
	glfwGetWindowSize( &width, &height );
	height = height > 0 ? height : 1;
	glViewport( 0, 0, width, height ); //set viewport
}

void GLFWCALL processGlobalKeys(int key,int action)
{
	//include timer here
	static double lastTime=glfwGetTime();
	double curTime=glfwGetTime();


	if((curTime-lastTime) > 0.1f) {

		if(glfwGetKey(GLFW_KEY_F1)) g_visualize=!g_visualize;
		if(glfwGetKey('\'')) g_enableGlow=!g_enableGlow;
		if(glfwGetKey(GLFW_KEY_F4)) toggleRetrace();
		if(glfwGetKey(GLFW_KEY_F5)) camType=CAM_MOUSEPAN;
		if(glfwGetKey(GLFW_KEY_F6)) camType=CAM_FOLLOW;
		if(glfwGetKey(GLFW_KEY_F7)) camType=CAM_FIRSTPERSON;
		if(glfwGetKey(GLFW_KEY_F10)) {
			int width,height;
			glfwGetWindowSize( &width, &height );
			printf("saving screenshot\n");
			//screenshot(width,height,"screenshot.jpg",100);
		}
		if(glfwGetKey(GLFW_KEY_F11)) gRenderUserData=!gRenderUserData;
		if(glfwGetKey(GLFW_KEY_F12)) gFreeze=!gFreeze;
		
		lastTime=curTime;
	}
}


//test of evolutionary robotics simulator

#include "base/Commandline.h"
#include "base/system.h"
#include "graphics/graphics.h"
#include "physics.h"
#include "testobjects.h"
#include "machines/quadromachine.h"


//par ting mangler
//somehow does not seem to work with elite? 

//-shader or similar eyecandy
//-fix normals?
//-save elite history  (kult å kunne lage video av elitehistorie)
//legg til slik at når man velger en maskin i testobjects får man peker til en modul som followcam kan følge
//  sånn at man kan vise fram maskinen med followcam
// -> kanskje forbedre dette: gi tilbake 2 vektorer fra en maskin (slik at follow kan være tilpsset hver enkelt maskin)?? 
// da slipper man også å "dele ut" parts
//kunne være interessant å ha params som ikke er evolverbare men allikevel params
//for cleanup: remove point3d and only use vec3 or nxvec3
//lage global default material så man slipper shaderkode rundt om kring i objects
//bedre rendering (shader) på cloth
//sette farge til shader!
//grafikkting i evolutionharness må ikke være duplikat av kode fra graphics/testobjects... kapsle inn og putte i graphics


int main(int argc, char *argv[])
{
	CommandLine cmdLineArgs;
	cmdLineArgs.parse(argc,argv);
	cmdLineArgs.printArguments();

	if(cmdLineArgs.noGUI)
		g_visualize=false;

	srand(3213212);
	//initWindow(640,480,false,false);
	initWindow(960,540,false,false);
	//initWindow(1280,720,false,true);
	initRendering();
	

	//*
	QuadroEvolutionHarness quadroEvo(42);
	quadroEvo.evolve(false);
	quadroEvo.printStatistics();
	/*/
	quadroLoop(cmdLineArgs.inFileName,cmdLineArgs.outFileName,cmdLineArgs.loopPlayback);
	//*/

	terminateGraphics();
	glfwTerminate();
	terminatePhysics();
}

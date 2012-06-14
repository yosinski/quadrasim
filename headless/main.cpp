//test of robotics simulator
#include "base/Commandline.h"
#include "physics.h"
#include "testobjects.h"
#include "machines/quadromachine.h"

int main(int argc, char *argv[])
{
	CommandLine cmdLineArgs;
	cmdLineArgs.parse(argc,argv);
	cmdLineArgs.printArguments();

	quadroLoop(cmdLineArgs.inFileName,cmdLineArgs.outFileName,cmdLineArgs.loopPlayback);
	terminatePhysics();
}

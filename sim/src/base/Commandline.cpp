#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include "wingetopt.h"
#else
#include <unistd.h>
#endif

#include "Commandline.h"

CommandLine::CommandLine() 
{
	noGUI=false;
	loopPlayback=false;
	useHardware=false;
}

void CommandLine::printUsage() 
{
	fprintf(stderr, "QuadraTot simulator usage\n");
	fprintf(stderr, "Arguments: [-i InputFile -o OutputFile -n -l -h]\n");
	fprintf(stderr, "\tn: No GUI\n");
	fprintf(stderr, "\tl: loop playback of control file\n");
	fprintf(stderr, "\th: playback also on robot\n");
	fprintf(stderr, "\tInputFile: Control file name\n");
	fprintf(stderr, "\tOutputFile: Output log file name\n");
}

void CommandLine::parse(int argc, char *argv[]) 
{
	int c;
	int errflg = 0;
	//extern char *optarg;
	//extern int optopt;
	char *inFile = NULL;
	char *outFile = NULL;

	while ((c = getopt(argc, argv, "i:o:nlh")) != -1) {
		switch (c) {
		case 'i':
			inFile = optarg;
			break;
		case 'o':
			outFile = optarg;
			break;
		case 'n':
			noGUI=true;
			break;
		case 'l':
			loopPlayback=true;
			break;
		case 'h':
			useHardware=true;
			break;
		case ':':   // arguments which require an argument but have none
			fprintf(stderr, "Option -%c requires an argument\n", optopt);
			errflg++;
			break;
		case '?':
			fprintf(stderr, "Unrecognized option: -%c\n", optopt);
			errflg++;
			break;
		}
	}

	if (errflg) {
		printUsage();
		exit(2);
	}

	if (inFile != NULL)
		inFileName.assign(inFile);
	if (outFile != NULL)
		outFileName.assign(outFile);


}

void CommandLine::printArguments()
{
	printf("display mode: %s\n", noGUI ? "console" : "visualize");
	printf("loop playback: %s\n", loopPlayback ? "yes" : "no");
	printf("playback on robot hardware: %s\n", useHardware ? "yes" : "no");
	if(!inFileName.empty())
		printf("using control commands from %s\n",inFileName.c_str());
	if(!outFileName.empty())
		printf("saving log data to %s\n",outFileName.c_str());
		
}
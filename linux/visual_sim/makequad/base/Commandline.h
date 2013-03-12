#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include <stdio.h>
#include <string>

struct CommandLine {
	bool noGUI; // -n option
	bool loopPlayback; // -l option
	std::string inFileName; // -i option
	std::string outFileName; // -o option
	 

	CommandLine();
	void printUsage();
	void parse(int argc, char *argv[]);
	void printArguments();
};


#endif

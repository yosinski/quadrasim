#ifndef FLOATPARAMMULTIVALLIST_H
#define FLOATPARAMMULTIVALLIST_H

#include "floatparamlist.h"
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>


struct FloatParamMultiValList {
	std::map<std::string, std::vector<FloatParam> > paramList;

	float getValue(const char* name,unsigned elementNumber=0);
	void setParams(const char* name,FloatParam p,unsigned numParams=1); //maybe change name of this later
	//void setSpecificParam(const char* name,FloatParam p,unsigned paramNumber); //resets a param in a given position
	void addParam(const char* name,FloatParam p); //adds a param to a given name (can be new)
	//setnumparams(name,numparams)?
	//setparam(name,paramno,param)?

	void print();
	int loadFromFile(const char *fileName);
	int loadFromStream(std::ifstream &inFile);
};


#endif
#include "floatparammultivallist.h"
#include "system.h"

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>

void FloatParamMultiValList::setParams(const char* name,FloatParam p,unsigned numParams/*=1*/)
{
	/*if(paramList.find(name)!=paramList.end()) {
		char errStr[200];
		sprintf(errStr,"value already in param list!: %s\n",name);
		systemError(errStr);
	}*/

	//if already there: overwriting
	paramList[name].assign(numParams,p);
}



void FloatParamMultiValList::addParam(const char* name,FloatParam p)
{
	paramList[name].push_back(p);
}


float FloatParamMultiValList::getValue(const char* name,unsigned elementNumber/*=0*/) 
{
	if(paramList.find(name)==paramList.end()) {
		char errStr[200];
		sprintf(errStr,"could not find value in param list!: %s\n",name);
		systemError(errStr);
	}
	else {
		if(elementNumber>=paramList[name].size())
			systemError("element index too high");
		else
			return (paramList[name])[elementNumber].val;
	}

	return 0;
}



void FloatParamMultiValList::print()
{
	printf("%d parameter names\n",paramList.size());


	std::map<std::string, std::vector<FloatParam> >::iterator it;
	for(it=paramList.begin(); it!=paramList.end(); it++) {
		std::vector<FloatParam>::iterator jt;
		for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
			printf("%s: val: %.2f min: %.2f max: %.2f precision: %d\n",it->first.c_str(),jt->val,jt->minVal,jt->maxVal,jt->precision);
		}
	}
}


int FloatParamMultiValList::loadFromFile(const char *fileName) 
{
	std::ifstream inFile(fileName,std::ios::in);
	if(!inFile) {
		printf("could not open params file\n");
		return -1;
	}

	loadFromStream(inFile);

	printf("loaded params (from file):\n");
	print(); //for debugging
	return 0;
}

int FloatParamMultiValList::loadFromStream(std::ifstream &inFile) 
{
	//assumes there are enough float values stored in the right order
	if(!inFile)
		return -1;

	std::map<std::string, std::vector<FloatParam>>::iterator it;
	for(it=paramList.begin(); it!=paramList.end() && !inFile.eof(); it++) {
		//inFile >> it->second.val;
		std::vector<FloatParam>::iterator jt;
		for(jt=it->second.begin(); jt!=it->second.end(); jt++) {
			inFile >> jt->val;
		}

	}
	return 0;
}


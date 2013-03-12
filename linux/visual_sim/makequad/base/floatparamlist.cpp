#include "floatparamlist.h"
#include "system.h"

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

//move elsewhere later
FloatParam::FloatParam(float value, float minValue, float maxValue) 
{
	val=value;
	minVal=minValue;
	maxVal=maxValue;
	precision=8;
}

FloatParam::FloatParam(float value, float minValue, float maxValue, int bitPrecision) {
	val=value;
	minVal=minValue;
	maxVal=maxValue;
	precision=bitPrecision;
}
	
FloatParam::FloatParam() {
	val=minVal=maxVal=0;
	precision=8;
}

float FloatParamList::getValue(const char* name) 
{
	if(paramList.find(name)==paramList.end()) {
		char errStr[200];
		sprintf(errStr,"could not find value in param list!: %s\n",name);
		systemError(errStr);
	}
	else
		return paramList[name].val;

	return 0;
}



void FloatParamList::print()
{
	printf("%d parameters\n",paramList.size());

	std::map<std::string, FloatParam>::iterator it;
	for(it=paramList.begin(); it!=paramList.end(); it++) {
		//printf("%s: val: %.2f min: %.2f max:%.2f\n",it->first.c_str(),it->second.val,it->second.minVal,it->second.maxVal);
		printf("%s: val: %.2f min: %.2f max: %.2f precision: %d\n",it->first.c_str(),it->second.val,it->second.minVal,it->second.maxVal,it->second.precision);
	}
}


int FloatParamList::loadFromFile(const char *fileName) 
{
	//assumes there are enough float values stored in the right order

	std::ifstream inFile(fileName,std::ios::in);
	if(!inFile) {
		printf("could not open params file\n");
		return -1;
	}

	std::map<std::string, FloatParam>::iterator it;
	int i=0;
	for(it=paramList.begin(); it!=paramList.end() && !inFile.eof(); it++) {
		inFile >> it->second.val;
		i++;
	}
	printf("loaded params (from file):\n");
	print(); //for debugging
	return 0;
}


#ifndef FLOATPARAMLIST_H
#define FLOATPARAMLIST_H

#include <map>
#include <string>
#include <fstream>
#include <iostream>

//move elsewhere later
struct FloatParam {
	float val;
	float minVal;
	float maxVal;
	int precision;

	FloatParam(float value, float minValue, float maxValue);
	FloatParam(float value, float minValue, float maxValue, int bitPrecision);
	FloatParam();
};

struct FloatParamList {
	std::map<std::string, FloatParam> paramList;

	float getValue(const char* name);
	void print();
	int loadFromFile(const char *fileName);
};


#endif
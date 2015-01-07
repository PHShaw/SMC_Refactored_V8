/*
 * utilities.h
 *  * Place to put common methods
 *  Created on: 23 Aug 2011
 *      Author: icub
 */


#ifndef UTILITIES_CPP
	#define UTILITIES_CPP


#ifndef PI
	#define PI 3.14159265
#endif

#include <stdlib.h>		//provides rand(), srand(), atexit(), abs(), and type size_t
#include <iostream>
#include <sstream>
#include <math.h>

#include "params_config.h"

/**
 * Returns a psudo random number between 0 and range (not including range)
 */
int randGenerator(const int range);
std::string appendInt(std::string s, int i);
double toRadians(double degs);
double toDegrees(double rads);

double* babbleGenerator(int noSamples, double babbleMin, double babbleMax);

//const double PI = 3.14159265;
#endif

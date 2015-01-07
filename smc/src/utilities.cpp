/*
 * utilities.cpp
 * Place to put common methods
 *
 *  Created on: 3 Jun 2011
 *      Author: icub
 */



#include "utilities.h"
/**
 * Returns a psudo random number between 0 and range (not including range)
 */
int randGenerator(const int range)
{
	if(range==0)
		return 0;
	return rand() % range;
}


std::string appendInt(std::string s, int i)
{
	std::ostringstream oss;
	oss << s << i;
	return oss.str();
}


double toRadians(double degs)
{
	return degs*PI/180;

}

double toDegrees(double rads)
{
	return rads*180/PI;
}

double* babbleGenerator(int noSamples, double babbleMin, double babbleMax)
{
	double interval = (PI*2/(noSamples-1));

	double* array = new double[noSamples];
	double r;

	for(int i=0; i<noSamples; i++)
	{
		r = ((double)rand() / RAND_MAX)/2 - 0.25;	//generate a random no. between -0.25 and 0.25
		array[i] = sin(i*interval) + r;
	}

	double babbleRange = (babbleMax-babbleMin)/2;
	double zeroOffset = babbleRange + babbleMin;

	//apply a random shift to the starting point in the array
	int startingPoint = randGenerator(noSamples);
	double* shifted = new double[noSamples];
	for(int i=0; i<noSamples; i++)
	{
		int x = (i+startingPoint) % noSamples;
		shifted[i] = babbleRange*array[x] + zeroOffset;
	}

	return shifted;
}

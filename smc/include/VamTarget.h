/*
 * VamTarget.h
 *
 *  Created on: 25 May 2013
 *      Author: icub
 */

#ifndef VAMTARGET_H_
#define VAMTARGET_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

#include <utilities.h>
#include <vector>
#include <set>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

class VamTarget
{
public:
	VamTarget();
	virtual ~VamTarget();

	void getVergence(int x, int y, int* xl, int* yl, int* xr, int* yr);
	void getFixatedObject(int* id, int* noFeatures);	//returns object ID
	void getPastObjectLocation(int id, int* x, int* y);		//returns true if object found

	int getObjectCount();
	bool getRandomObject(float* x, float* y);
	yarp::os::Bottle* getBottle(){return VAMtarg;}

private:
	void initYarp();
	std::set<int> segments;
	yarp::os::BufferedPort<yarp::os::Bottle> VAMportIn;
	yarp::os::BufferedPort<yarp::os::Bottle> VAMportOut;
	yarp::os::Bottle* VAMtarg;

	const static int camera = 1;
};

#endif /* VAMTARGET_H_ */

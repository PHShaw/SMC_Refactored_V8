/*
 * Target.h
 * Note, logs are initialised separately.
 *
 *  Created on: 23 Aug 2011
 *      Author: icub
 */

#ifndef TARGET_H
#define TARGET_H


#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

#include <utilities.h>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

class Target{
public:
	Target();
	Target(std::string robot);
	Target(bool uu);
	bool initTarget(std::string robot);
	bool getTarget(double* targX, double* targY);	//return first colour listed
	bool getTarget(double* targX, double* targY, const std::string colour);	//look for specific colour

	std::string getNearestObject(double* targX, double* targY, double* dist);
	std::vector<std::string> getNearestObjects();

	std::string getNearestTo(const int x, const int y, double* targX, double* targY, double* dist);

	yarp::os::Bottle* getAllTargets();
	bool targetCentred(double* targX, double* targY);
	bool targetCentred();
	bool targetCentred(double* targX, double* targY, const std::string colour);
	bool targetVisible();
	std::string getColour();
	int getSize(){return size;}


	bool getLeft(double* targX, double* targY, std::string colour);


	bool matchColour(const std::string colourTest);
	bool fovea(double x, double y, double* dist);
	void initLog(std::string path);
	void logSaccadeBreak(int saccadeCount);
	void closeLog();


	bool uuGetTarget(double* targX, double* targY);

	void getLastTargs(double* lastX, double* lastY);


private:
	yarp::os::BufferedPort<yarp::os::Bottle> porttargetsleft;
	yarp::os::BufferedPort<yarp::os::Bottle> porttargetsright;
	yarp::os::BufferedPort<yarp::os::Bottle> portUU;
	yarp::os::Bottle* target;
//	yarp::os::Bottle* target;
	bool visible;
	std::ofstream targetlogfile;

	bool ulster;
	bool requestROI;	//Region of interest
	bool sentFixate;
	double lastX, lastY;

	const static double requestNewROI = 1;
	const static double whereIsROI = 2;
	const static double fixated = 3;

	std::string colour;
	int size;
	bool centred;

};

const int elements=4;

const  std::string RED = "red";
const  std::string YELLOW = "yellow";
const  std::string GREEN = "green";
const  std::string BLUE = "blue";
const  std::string WHITE = "white";
const  std::string GRAY = "gray";
const  std::string HAND = YELLOW;


const std::string colours[4] = {YELLOW, RED, GREEN, BLUE};



#endif

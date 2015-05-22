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
#include <map>
#include <string.h>

#include <utilities.h>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

class Target // : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
	Target();
	bool initTarget(std::string robot);
	bool getTarget(double* targX, double* targY);	//return first colour listed
	bool getTarget(double* targX, double* targY, const std::string colour);	//look for specific colour

	std::string getNearestObject(double* targX, double* targY, double* dist);
	std::vector<std::string> getNearestObjects();

	std::string getNearestTo(const int x, const int y, double* targX, double* targY, double* dist);

	//virtual void onRead(yarp::os::Bottle& b);

	yarp::os::Bottle* getAllTargets();
	bool targetCentred(double* targX, double* targY);
	bool targetCentred();
	bool targetCentred(double* targX, double* targY, const std::string colour);
	bool targetVisible();
	std::string getColour();
	int getSize(){return size;}

	int getNumTargets(){return numTargets;}
	int getNumTargets(std::string type);

	bool getLeft(double* targX, double* targY, std::string colour);


	bool matchColour(const std::string colourTest);
	bool fovea(double x, double y, double* dist);
	void initLog(std::string path);
	void logSaccadeBreak(int saccadeCount);
	void closeLog();



	void getLastTargs(double* lastX, double* lastY);


private:

	std::string processColour(int index, double* targX, double* targY, int* size, bool advanced=false);
	std::string processMotion(int index, double* targX, double* targY, int* size, bool advanced=true);


	yarp::os::BufferedPort<yarp::os::Bottle> porttargetsleft;
	yarp::os::BufferedPort<yarp::os::Bottle> porttargetsright;
	yarp::os::BufferedPort<yarp::os::Bottle> portUU;
	yarp::os::Bottle* target;
//	yarp::os::Bottle* target;
	bool visible;
	std::ofstream targetlogfile;



	double lastX, lastY;


	std::string colour;
	int size;
	bool centred;

	std::map<std::string, int> elementsPerTargetTypes;
	std::map<std::string, int> numVisibleTargets;
	int colourElements; // "colour" [colour] [x] [y] [pixels]
	int numTargets;
};

const int shapeElements = 7;  //"shape" [square|circle|triangle] [colour] [x] [y] [width] [height]
const int edgeElements = 12;  //"edge" [x_centre] [y_centre] [num_pts=4] [x1][y1][x2][y2]... (default 4 points)
const int motionElements = 6; //"motion" [speed px/s] [x0] [y0] [x1] [y1]


const  std::string RED = "red";
const  std::string YELLOW = "yellow";
const  std::string GREEN = "green";
const  std::string BLUE = "blue";
const  std::string WHITE = "white";
const  std::string GRAY = "gray";
const  std::string HAND = YELLOW;


const std::string colours[4] = {YELLOW, RED, GREEN, BLUE};



#endif

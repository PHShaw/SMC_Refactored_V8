/*
 * VamTarget.cpp
 *
 *  Created on: 25 May 2013
 *      Author: icub
 */

#include "../include/VamTarget.h"

VamTarget::VamTarget()
{
	initYarp();
}

VamTarget::~VamTarget()
{
	yarp::os::Bottle& SMCDMCout = VAMportOut.prepare();
	SMCDMCout.clear();
	SMCDMCout.addString("exit");
	VAMportOut.write();

	VAMportIn.close();
    VAMportOut.close();
}

void VamTarget::getVergence(int x, int y, int* xl, int* yl, int* xr, int* yr)
{
	yarp::os::Bottle& SMCDMCout = VAMportOut.prepare();
	SMCDMCout.clear();
	SMCDMCout.addString("camera");
	SMCDMCout.addInt(camera);//0 for left camera and 1 for right camera
	SMCDMCout.addInt(x);
	SMCDMCout.addInt(y);
	VAMportOut.write();
	std::cout << "sending coordinates(random for now) ("<<x<<") ("<<y<<")"<< std::endl;

	VAMtarg = VAMportIn.read(true);

	std::string dummy;
	dummy = VAMtarg->get(0).asString();
//	std::cout<<"received: "<<dummy<<std::endl;
	*xl = VAMtarg->get(1).asInt();
	*yl = VAMtarg->get(2).asInt();
	*xr = VAMtarg->get(3).asInt();
	*yr = VAMtarg->get(4).asInt();
	std::cout<<"received coordinates : ("<<*xl<<") ("<<*yl<<") & ("<<*xr<<") ("<<*yr<<")"<<std::endl;
}

void VamTarget::getFixatedObject(int* id, int* noFeatures)
{
	yarp::os::Bottle& SMCDMCout = VAMportOut.prepare();
	SMCDMCout.clear();
	SMCDMCout.addString("sendobject");
	SMCDMCout.addString("camera");
	SMCDMCout.addInt(camera);//0 for left camera and 1 for right camera
	VAMportOut.write();

	VAMtarg = VAMportIn.read(true);
	*id = VAMtarg->get(2).asInt();
	*noFeatures = VAMtarg->get(4).asInt();
}

void VamTarget::getPastObjectLocation(int id, int* x, int* y)
{
	yarp::os::Bottle& SMCDMCout = VAMportOut.prepare();
	SMCDMCout.clear();
	SMCDMCout.addString("check");
	SMCDMCout.addString("camera");
	SMCDMCout.addInt(camera);//0 for left camera and 1 for right camera
	SMCDMCout.addString("objectID");
	SMCDMCout.addInt(id);
	VAMportOut.write();

	VAMtarg = VAMportIn.read(true);
	*x = VAMtarg->get(1).asInt();
	*y = VAMtarg->get(2).asInt();
}

int VamTarget::getObjectCount()
{
	yarp::os::Bottle& SMCDMCout = VAMportOut.prepare();
	SMCDMCout.clear();
	SMCDMCout.addString("segment");
	SMCDMCout.addString("camera");
	SMCDMCout.addInt(camera);
	int x = 160;
	int y = 120;
	SMCDMCout.addInt(x);	//Any random location can be sent
	SMCDMCout.addInt(y);
	VAMportOut.write();

	VAMtarg = VAMportIn.read(true);
	int counter = 0;
	std::string dummy;

	dummy = VAMtarg->get(counter++).asString();		// "vamsegment"
	dummy = VAMtarg->get(counter++).asString();		// "feature_count"
	int featureCount = VAMtarg->get(counter++).asInt();
	dummy = VAMtarg->get(counter++).asString();		// "feature_segments"

	//Format of segments is x1, y1, s1, ... xn, yn, sn
	//where (x,y) are coordinates of a feature
	// and  s is the segment it is in
	//In this function, want to know how many segments there are, i.e. the max segment number

	segments.clear();
	int segMax = 0;
	for (int i = 0; i<featureCount; i++) {
		int segmentIdx = VAMtarg->get(3*i+counter+2).asInt();
		segments.insert(segmentIdx);
		if(segmentIdx > segMax)
			segMax = segmentIdx;
	}
	return segments.size();
}

bool VamTarget::getRandomObject(float* x, float * y)
{
	//Coordinates of features in each segment are returned
	//need to obtain a single coordinate for an object, which is an average of coords for a random segment

	int segmentCount = getObjectCount();
	std::cout << "There are " << segmentCount << " segments detected" << std::endl;
	int randSegment = randGenerator(segmentCount-2)+2;
	std::cout << "Selected segment " << randSegment << std::endl;
	std::set<int>::iterator it;

	int i=0;
	for(it=segments.begin(); it!=segments.end(); ++it, i++)
	{
		std::cout << *it << " ";
		if(i == randSegment)
		{
			randSegment = *it;
			break;
		}

	}
	std::cout << std::endl;

	int objectFeatures=0;
	float avgx=0;
	float avgy=0;

	int featureCount = VAMtarg->get(2).asInt();
	int counter = 4;
	for (int i = 0; i<featureCount; i++) {
		int segmentIdx = VAMtarg->get(3*i+counter+2).asInt();
		std::cout << segmentIdx << " ";
		if(segmentIdx == randSegment)
		{
			objectFeatures ++;
			avgx += VAMtarg->get(3*i+counter).asInt();
			avgy += VAMtarg->get(3*i+counter+1).asInt();
		}
	}
	std::cout << std::endl;
	std::cout << "objectFeatures: " << objectFeatures << ", totals: " << avgx << ", " << avgy << std::endl;
	if(objectFeatures==0)
		return false;
	avgx = avgx/objectFeatures;
	avgy = avgy/objectFeatures;

	*x = avgx;
	*y = avgy;
	return true;
}

void VamTarget::initYarp()
{
	yarp::os::Network yarp;

	VAMportIn.open("/SMCDMC/in");
	VAMportOut.open("/SMCDMC/out");

	yarp.connect("/SMCDMC/out", "/vamprocess/in");
	yarp.connect("/vamprocess/out","/SMCDMC/in");

}

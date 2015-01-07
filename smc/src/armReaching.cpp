/*
 * armReaching.cpp
 *  TODO: create arm controller, learning using virtual skin and hand flat over surface?
 *  Created on: 17 Aug 2011
 *      Author: icub
 */

#include "armReaching.h"

using namespace std;

armReaching::armReaching(GazeMap* pgm)
{
	gm = pgm;

	armCont = new armController(true);	//true = hands only, false = whole arm
	cout << "Arm controller initialised" << endl;

	openLogs();
}

armReaching::~armReaching()
{
	closeLogs();
}


//can make use of depth and angle, so need methods for looking up fields with those


bool armReaching::getReachField(double depth, double angle, ReachField** reach)
{
	*reach = gm->getReachField(depth, angle);
	logEntry("1. Obtained reach field:");
	logField(*reach);
	if((*reach)->getXcoord()==0 && (*reach)->getYcoord()==0)
		return false;
	else
		return true;
}

bool armReaching::getReachField(ReachField** reach, bool rightArm)
{
	double* position = new double[16];
	armCont->getCurrentPosition(position, rightArm);
	logEntry("2a. Looking up position:");
	logPosition(position);
	*reach = gm->getReachField(position, rightArm);
	logEntry("2a. Obtained reach field:");
	logField(*reach);
	if((*reach)->getXcoord()==0 && (*reach)->getYcoord()==-0)
	{
		gm->addReachField(position, rightArm);
		logEntry("2b. Looking up position:");
		logPosition(position);
		*reach = gm->getReachField(position, rightArm);
		logEntry("2b. Obtained reach field:");
		logField(*reach);
		if((*reach)->getXcoord()==0 && (*reach)->getYcoord()==-0)
			return false;
		else
			return true;
	}
	else
		return true;
}

bool armReaching::addReachField(bool rightArm)
{
	if(!isReachField(rightArm))
	{
		double* position = new double[16];
		armCont->getCurrentPosition(position, rightArm);
		gm->addReachField(position, rightArm);
		return true;
	}
	else
		return false;

}

bool armReaching::isReachField(bool rightArm)
{
	double* position = new double[16];
	armCont->getCurrentPosition(position, rightArm);
	ReachField* reach = gm->getReachField(position, rightArm);
	if(reach->getXcoord()!=0 && reach->getYcoord()!=0)
		return true;
	else
		return false;
}

ReachField* armReaching::getReachField(bool rightArm){
	double* position = new double[16];
	armCont->getCurrentPosition(position, rightArm);
	logEntry("3. Looking up position:");
	logPosition(position);
	return gm->getReachField(position, rightArm);;
}


void armReaching::getCoordinates(double* depth, double* angle, bool rightArm)
{
	*depth = armCont->getDepth(rightArm);
	*angle = armCont->getAngle(rightArm);

	logCoords(*depth, *angle);
}





bool armReaching::learnReach(int rightArm)
{
	armCont->babble(rightArm);
	double* position = new double[16];
	bool success;
	if(rightArm==armCont->BOTH)
	{
		//left then right
		success = armCont->getCurrentPosition(position, armCont->LEFT);
		if(success)
		{
			logEntry("Learning left arm pose");
			logPosition(position);
			gm->addReachField(position, armCont->LEFT);
		}

		bool success2 = armCont->getCurrentPosition(position, armCont->RIGHT);
		if(success2)
		{
			logEntry("Learning right arm pose");
			logPosition(position);
			gm->addReachField(position, armCont->RIGHT);
		}
		success &= success2;
	}
	else
	{
		success=armCont->getCurrentPosition(position, rightArm);
		if(success)
		{
			logEntry("Learning pose");
			logPosition(position);
			gm->addReachField(position, rightArm);
		}
	}
	return success;
}


int armReaching::learnReaching(int rightArm)
{
	int counter = 0;
	int failure;
	while(counter<50)
	{
		startReachLog(counter + failure);
		bool success = learnReach(rightArm);
		if(success)
			counter ++;
		else
			failure ++;

		endReachLog(success);
	}
	return counter;
}


int armReaching::learnReaching(bool rightArm)
{
	int counter = 0;
	int failure = 0;
	while(counter<50)
	{
		startReachLog(counter + failure);
//		int arm = randGenerator(3);

		bool success = learnReach(rightArm);
		if(success)
			counter ++;
		else
			failure ++;

		endReachLog(success);
	}
	return counter;
}


void armReaching::goToReachField(ReachField* reach)
{
	if(reach->getXcoord()==0 && reach->getYcoord()==0)
	{
		cout << "Invalid reach field: " << reach << endl;
		return;
	}
	bool rightArm;
	ReachConfig* rc = reach->getPreferredReach(&rightArm);
	double* position = new double[16];
	rc->toArray(position);
	if(rightArm)
		armCont->armAction(armCont->REACH_RIGHT,position);
	else
		armCont->armAction(armCont->REACH_LEFT,position, rightArm);

}


void armReaching::goToReachConfig(ReachConfig* rc)
{
	bool rightArm = rc->rightArm;
	double* position = new double[16];
	rc->toArray(position);
	if(rightArm)
		armCont->armAction(armCont->REACH_RIGHT,position);
	else
		armCont->armAction(armCont->REACH_LEFT,position);

}

void armReaching::toRest()
{
	armCont->armAction(armCont->REST,HOME_POSE, armCont->LEFT);
	armCont->armAction(armCont->REST,HOME_POSE, armCont->RIGHT);
}

void armReaching::toRest(bool rightArm)
{
	if(rightArm)
		armCont->armAction(armCont->REST,HOME_POSE, armCont->RIGHT);
	else
		armCont->armAction(armCont->REST,HOME_POSE, armCont->LEFT);
}

//Logging  methods:
void armReaching::openLogs()
{
	string fullpath = "../data/armReachlog.txt";

	logfile.open(fullpath.c_str());

}
void armReaching::logCoords(double depth, double angle)
{
	logfile << "depth: " << depth << " angle: " << angle << endl;
}
void armReaching::logEntry(string message)
{
	cout << message << endl;
	logfile << message << endl;
}
//void armReaching::logEntry(std::string message,double x,double y);
void armReaching::logField(const Field* field)
{
	cout << *field << endl;
	logfile << *field << endl;
}

void armReaching::logPosition(const double* position)
{
	logfile << "position: ";
	for (int i=0; i<16; i++)
	{
		logfile << " " << position[i];
	}
	logfile << endl;
}

void armReaching::startReachLog(int counter)
{
	logfile << "*******************************" << endl;
	logfile << "Starting Reach: " << counter << endl;
}
void armReaching::endReachLog(bool success)
{
	if(success)
	{
		logfile << "Successfully learnt reach" << endl;
	}
	else
	{
		logfile << "Failed to learn reach" << endl;
	}
	logfile << "*******************************" << endl;
}
void armReaching::closeLogs()
{
	logfile << "**************END**************" << endl;
	logfile.close();
}





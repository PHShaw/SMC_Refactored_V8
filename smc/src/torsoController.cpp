/*
 * torsoController.cpp
 *
 *  Created on: 10 Nov 2011
 *      Author: icub
 */

#include "torsoController.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


torsoController::torsoController(string robot)
{
	initYarp(robot);

	string fullpath = params.path + "torsomotorlog.txt";
	motorlogfile.open(fullpath.c_str());
	motorlogfile << "yaw pitch" << endl;
	toRest();


}


bool torsoController::initYarp(string robot)
{
	string torbit = "/";
	torbit += robot;
	torbit += "/torso";

	//Set up yarp network to receive target data and send motor commands:
	yarp::os::Network yarp;
	Property options;
	options.put("device", "remote_controlboard");

	string hport = "/torsomove";
	if(robot.compare("icubSim")==0)
		hport += "Sim";
	options.put("local", hport.c_str());

	options.put("remote", torbit.c_str());


	motordriver=new PolyDriver;
	motordriver->open(options);


	if (!motordriver->isValid())
	{
		cout << "Failed to open device\n";
		return false;
	}
	bool ok;
	ok = motordriver->view(pos);
	ok &= motordriver->view(enc);


	if (!ok)
	{
		cout << "Failed to obtain position or encoder interface" << endl;
		return false;
	}

	pos->setRefSpeed(torsoYaw,7);
	pos->setRefSpeed(torsoPitch,7);

	pos->positionMove(0, 0);
	pos->positionMove(1, 0);
	pos->positionMove(2, 0);


	return true;
}


bool torsoController::stationary()
{
	bool motionCompleted = false;
	bool x=false, y=false;
	bool success;
	success = pos->checkMotionDone(torsoYaw, &x);
	success &= pos->checkMotionDone(torsoPitch, &y);

	if(!success)
	{
		cout << "Error whilst attempting to check if torso motion done" << endl;
	}
	motionCompleted = x && y;
	if(motionCompleted)
	{
		cout << "torso motion done" << endl;
		return true;
	}
	else
	{
		return false;
	}


}


bool torsoController::move (const double x, const double y, bool block)
{
	bool inReach = true;
	//Check x and y are within range
	double checkedX, checkedY;
	if(x>xMax)
	{
		checkedX = xMax;
		inReach = false;
	}
	else if(x<xMin)
	{
		checkedX = xMin;
		inReach = false;
	}
	else
		checkedX = x;

	if(y>yMax)
	{
		checkedY = yMax;
		inReach = false;
	}
	else if(y<yMin)
	{
		checkedY = yMin;
		inReach = false;
	}
	else
		checkedY = y;


	pos->positionMove(torsoYaw, checkedX);
	pos->positionMove(torsoPitch, checkedY);

	motorlogfile << checkedX << " " << checkedY << endl;

	while(block && !stationary())
	{
		Time::delay(0.1);
	}
	return inReach;
}


void torsoController::generateRandomPosition(double* x, double* y)
{
	double xRange = xMax + abs((int)xMin);
	double yRange = yMax + abs((int)yMin);

	*x = randGenerator((int)xRange) + xMin;
	*y = randGenerator((int)yRange) + yMin;

}

void torsoController::generateSmallRelativeMovement(int maxSize, double* x, double* y)
{
	generateRelativeMovement(maxSize, maxSize, x,y);
}


void torsoController::generateRelativeMovement(int xmaxSize, int ymaxSize, double* x, double* y)
{
	getCurrentPosition(x,y);
	int change = randGenerator(xmaxSize*2)-xmaxSize;
	cout << "x change: " << change << endl;
	*x += change;
	if(*x>xMax)
		*x=xMax;
	else if(*x<xMin)
		*x = xMin;

	change = randGenerator(ymaxSize*2)-ymaxSize;
	cout << "y change: " << change << endl;
	*y+= change;
	if(*y>yMax)
		*y=yMax;
	else if(*y<yMin)
		*y = yMin;
}


void torsoController::babble(bool block)
{
	double x=0,y=0;
	generateRelativeMovement(60,0, &x, &y);
	cout << "torso babbling to " << x << ", " << y << endl;
	move(x,y,block);

}


void torsoController::toRest()
{
	move(0,0);
}

bool torsoController::getCurrentPosition(double* x, double* y)
{
	bool success;
	success = enc->getEncoder(torsoYaw,x);
	success &= enc->getEncoder(torsoPitch,y);
	return success;
}

bool torsoController::getCurrentPosition(double* torsoMotorPos)
{
	bool success;
	success = enc->getEncoders(torsoMotorPos);
	return success;
}


bool torsoController:: isAtPosition(const double x, const double y)
{
	double curX, curY;
	getCurrentPosition(&curX, &curY);
	if((int)curX == (int)x && (int)curY == (int)y)
		return true;
	else
		return false;
}

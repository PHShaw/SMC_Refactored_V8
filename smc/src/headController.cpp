/*
 * headController.cpp
 *
 *  Created on: 6 Jun 2011
 *      Author: icub
 */

#include "headController.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


headController::headController(PolyDriver* pmotordriver)
{
	motordriver = pmotordriver;
	bool ok;
	ok = motordriver->view(pos);
	ok &= motordriver->view(enc);

	if (!ok)
	{
		cout << "Failed to obtain position or encoder interface" << endl;
	}

	pos->setRefSpeed(headX,15);
	pos->setRefSpeed(headY,15);

	pos->positionMove(0, -38);
	pos->positionMove(1, 0);
	pos->positionMove(2, 0);

	toRest();
}


bool headController::stationary()
{
	bool motionCompleted = false;
	bool x=false, y=false;
	bool success;
	success = pos->checkMotionDone(headX, &x);
	success &= pos->checkMotionDone(headY, &y);

	if(!success)
	{
		cout << "Error whilst attempting to check if head motion done" << endl;
	}
	motionCompleted = x && y;
	if(motionCompleted)
	{
		cout << "neck motion done" << endl;
		return true;
	}
	else
	{
		return false;
	}

//
//	double testX1, testY1, testX2, testY2;
//	bool success;
//	success = getCurrentPosition(&testX1, &testY1);
//	Time::delay(0.2);
//	success &= getCurrentPosition(&testX2, &testY2);
//	if(!success)
//	{
//		cout << "Unable to obtain one or more head position encoders" << endl;
//		return false;
//	}
//
//	if((int)testX1!=(int)testX2 && (int)testY1!=(int)testY2)
//		return false;
//	else
//		return true;
}


void headController::move (const double x, const double y, bool block)
{
	//Check x and y are within range
	double checkedX, checkedY;
	if(x>xMax)
		checkedX = xMax;
	else if(x<xMin)
		checkedX = xMin;
	else
		checkedX = x;

	if(y>yMax)
		checkedY = yMax;
	else if(y<yMin)
		checkedY = yMin;
	else
		checkedY = y;


	pos->positionMove(headX, checkedX);
	pos->positionMove(headY, checkedY);

	while(block && !stationary())
	{
		Time::delay(0.1);
	}
}


void headController::generateRandomPosition(double* x, double* y)
{
	//Actual positions generated
	double xRange = xMax + abs((int)xMin);
	double yRange = yMax + abs((int)yMin);

	*x = randGenerator((int)xRange) + xMin;
	*y = randGenerator((int)yRange) + yMin;


	//Relative positions generated:
	//Doesn't check if it is outside of movement ranges
//	int relX = randGenerator((int)xRange/2) - xRange/4;
//	int relY = randGenerator((int)yRange/2) - yRange/4;
//	getCurrentPosition(x, y);
//	*x = *x + relX;
//	*y = *y + relY;


	//relative movements, with a minimum of 5 degrees
	//full x range: -35->35 = 70
	//full y range: -35->18 = 53
//
//	double xRange = (xMax-xMin)*(3/4);
//	double yRange = (yMax-yMin)*(3/4);
//	int relX = randGenerator((int)xRange-5);
//	int relY = randGenerator((int)yRange-5);
//
//	getCurrentPosition(x, y);
//	*x = *x + relX;
//	*y = *y + relY;
//
//	if(*x>xMax)
//		*x = xMax;
//	else if(*x<xMin)
//		*x = xMin;
//
//
//	if(*y>yMax)
//		*y = yMax;
//	else if(*y<yMin)
//		*y = yMin;


	//TODO Note: if you want to be really clever, consider the eye in head position when generating this movement
}

void headController::generateSmallRelativeMovement(double* x, double* y, double maxRange)
{
	double change = randGenerator(maxRange*2)-maxRange;
	*x += change;

	change = randGenerator(maxRange*2)-maxRange;
	*y += change;
}


void headController::babble(bool block)
{
	double x,y;
	generateRandomPosition(&x, &y);
	move(x,y,block);
	cout << "head babbling to " << x << ", " << y << endl;
}

void headController::smallbabble(bool block, double maxRange)
{
	double x,y;
	getCurrentPosition(&x, &y);
	generateSmallRelativeMovement(&x, &y, maxRange);
	move(x,y, block);
	cout << "head small babbling to " << x << ", " << y << endl;
}

void headController::toRest()
{
	move(0,0);
}
void headController::headDown()
{
	move(0,-30);
}

bool headController::getCurrentPosition(double* x, double* y)
{
	bool success;
	success = enc->getEncoder(headX,x);
	success &= enc->getEncoder(headY,y);
	return success;
}

bool headController::getCurrentPosition(double* headMotorPos)
{
	bool success;
	success = enc->getEncoders(headMotorPos);
	return success;
}


bool headController:: isAtPosition(const double x, const double y)
{
	double curX, curY;
	getCurrentPosition(&curX, &curY);
	if((int)curX == (int)x && (int)curY == (int)y)
		return true;
	else
		return false;
}

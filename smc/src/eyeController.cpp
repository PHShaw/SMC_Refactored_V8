/*
 * eyeController.cpp
 *
 *  Created on: 3 Jun 2011
 *      Author: icub
 */

#include "eyeController.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


eyeController::eyeController(PolyDriver* pmotordriver)
{
	motordriver = pmotordriver;
	bool ok;
	ok = motordriver->view(pos);
	ok &= motordriver->view(enc);

	if (!ok)
	{
		cout << "Failed to obtain position or encoder interface" << endl;
	}

	pos->setRefSpeed(eyeX,80);
	pos->setRefSpeed(eyeY,80);

	pos->positionMove(3, 0);
	pos->positionMove(4, 0);
	pos->positionMove(5, 0);

	toRest();
}


bool eyeController::stationary()
{
	bool motionCompleted = false;
	bool x=false, y=false;
	bool success;
	success = pos->checkMotionDone(eyeX, &x);
	success &= pos->checkMotionDone(eyeY, &y);

	if(!success)
	{
		cout << "Error whilst attempting to check if eye motion done" << endl;
	}
	motionCompleted = x && y;
	if(motionCompleted)
	{
		cout << "eye motion done" << endl;
		return true;
	}
	else
	{
//		cout << "still moving" << endl;
		return false;
	}


//	double testX1, testY1, testX2, testY2;
//
//	success = getCurrentPosition(&testX1, &testY1);
//	Time::delay(0.2);
//	success &= getCurrentPosition(&testX2, &testY2);
//	if(!success)
//	{
//		cout << "Unable to obtain one or more eye position encoders" << endl;
//		return false;
//	}
//
//	if((int)testX1!=(int)testX2 && (int)testY1!=(int)testY2)
//		return false;
//	else
//		return true;
}


void eyeController::move (const double x, const double y, bool block)
{
	//Check x and y are within range
	double checkedX, checkedY;
	if(x>xMax)
	{
		checkedX = xMax;
		printf("X changed from %.2f to %.2f\n", x, checkedX);
	}
	else if(x<xMin)
	{
		checkedX = xMin;
		printf("X changed from %.2f to %.2f\n", x, checkedX);
	}
	else
		checkedX = x;

	if(y>yMax)
	{
		checkedY = yMax;
		printf("Y changed from %.2f to %.2f\n", y, checkedY);
	}
	else if(y<yMin)
	{
		checkedY = yMin;
		printf("Y changed from %.2f to %.2f\n", y, checkedY);
	}
	else
		checkedY = y;


	pos->positionMove(eyeX, checkedX);
	pos->positionMove(eyeY, checkedY);

	while(block && !stationary())
	{
		Time::delay(0.1);
	}
}


void eyeController::verg(const double vergence, bool block)
{
	double checkedVerg;
	if(vergence > 50)
		checkedVerg = 50;
	else if(vergence < 0)
		checkedVerg = 0;
	else
		checkedVerg = vergence;

	pos->positionMove(5,vergence);
	bool success = false, v=false;
	while(block && !v)
	{
		success = pos->checkMotionDone(5, &v);
	}

}

void eyeController::generateRandomPosition(double* x, double* y)
{
	double xRange = xMax + abs((int)xMin);
	double yRange = yMax + abs((int)yMin);

	*x = randGenerator((int)xRange) + xMin;
	*y = randGenerator((int)yRange) + yMin;

}

void eyeController::generateSmallRelativeMovement(double* x, double* y, double maxRange)
{
	double change = randGenerator(maxRange*2)-maxRange;
	*x += change;

	change = randGenerator(maxRange*2)-maxRange;
	*y += change;
}


void eyeController::babble(bool block)
{
	double x,y;
	generateRandomPosition(&x, &y);
	move(x,y,block);
	cout << "eye babbling to " << x << ", " << y << endl;
}

void eyeController::smallbabble(bool block, double maxRange)
{
	double x,y;
	getCurrentPosition(&x, &y);
	generateSmallRelativeMovement(&x, &y, maxRange);
	move(x,y, block);
	cout << "eye small babbling to " << x << ", " << y << endl;
}

/**
 * blocks
 */
void eyeController::toRest()
{
	move(0,0);
	verg(0, false);
}

bool eyeController::getCurrentPosition(double* x, double* y)
{
	bool success;
	success = enc->getEncoder(eyeX,x);
	success &= enc->getEncoder(eyeY,y);
	return success;
}

bool eyeController::getCurrentPosition(double* headMotorPos)
{
	bool success;
	success = enc->getEncoders(headMotorPos);
	return success;
}

bool eyeController::getVergence(double* vergence)
{
	bool success;
	success = enc->getEncoder(5,vergence);
	return success;
}


bool eyeController:: isAtPosition(const double x, const double y)
{
	double curX, curY;
	getCurrentPosition(&curX, &curY);
	if((int)curX == (int)x && (int)curY == (int)y)
		return true;
	else
		return false;
}

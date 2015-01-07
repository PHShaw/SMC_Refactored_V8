/*
 * reachField.cpp
 *
 *  Created on: 2 Aug 2011
 *      Author: icub
 */

#include "ReachField.h"
using namespace std;


ostream& operator <<(ostream& outs, const ReachConfig& source)
{
	outs << "rotation: " <<source.rotation() << ", ";
	outs << "flexion: " << source.flexion();
	return outs;
}

bool operator ==(const ReachConfig& rc1, const ReachConfig& rc2)
{
	return  rc1.shoulderPitch==rc2.shoulderPitch &&
			rc1.shoulderRoll==rc2.shoulderRoll &&
			rc1.shoulderYaw==rc2.shoulderYaw &&
			rc1.elbow==rc2.elbow &&
			rc1.wristPitch==rc2.wristPitch;
}

ReachField::ReachField()
{
	leftReached = false;
    rightReached = false;

    x = 0;
	y = 0;
	index = 0;

	leftReaches.reserve(5);
	rightReaches.reserve(5);
	usage=0;
	radius = 0.5;
}


ReachField::ReachField(size_t fieldID, double depth, double angle, double* reach, bool rightArm, double rad)
{
    leftReached = false;
    rightReached = false;


	x = (float)depth;
	y = (float)angle;
	leftReaches.reserve(5);
	rightReaches.reserve(5);
	addReachConfiguration(reach, rightArm);
	index = fieldID;
	usage=1;
	radius = rad;
}


ReachField::~ReachField()
{

	leftReaches.clear();
	leftReaches.resize(1);

	rightReaches.clear();
	rightReaches.resize(1);
}


void ReachField::getReachDirection(double* depth, double* angle) const
{
	*depth = (double)x;
	*angle = (double)y;
}

void ReachField::addReachConfiguration(double* reach, bool rightArm)
{
	ReachConfig* rc = new ReachConfig(reach, rightArm);
	if(rightArm)
	{
		for(int i=0; i<rightReaches.size(); i++)
		{
			ReachConfig* r2 = rightReaches.at(i);
			if(*rc==*r2)
				return;
		}
		rightReaches.push_back(rc);
	}
	else
	{
		for(int i=0; i<leftReaches.size(); i++)
		{
			ReachConfig* r2 = leftReaches.at(i);
			if(*rc==*r2)
				return;
		}
		leftReaches.push_back(new ReachConfig(reach, rightArm));
	}
}



ReachConfig* ReachField::getLeftReach()
{
	double cost = 0;
	if(leftReaches.size()>0)
		return getReach(&leftReaches, &cost);
	else
		return new ReachConfig();
}

ReachConfig* ReachField::getRightReach()
{
	double cost = 0;
	if(rightReaches.size()>0)
		return getReach(&rightReaches, &cost);
	else
		return new ReachConfig();
}



//Bool changes to indicate which arm the preferred reach is for
ReachConfig* ReachField::getPreferredReach(bool *rightArm)
{
	double leftCost, rightCost;
	ReachConfig* leftReach = getReach(&leftReaches, &leftCost);
	ReachConfig* rightReach = getReach(&rightReaches, &rightCost);

	printf("There are %i left reaches and %i right reaches\n", leftReaches.size(), rightReaches.size());
	if(min(leftCost, rightCost) == rightCost)
	{
		*rightArm = true;
		return rightReach;
	}
	else
	{
		*rightArm = false;
		return leftReach;
	}

}


ReachConfig* ReachField::getReach(vector<ReachConfig*> *reaches, double* pCost)
{
	ReachConfig* armReach = new ReachConfig();
	ReachConfig* temp;
	double minCost;	//Split between rotation and flexion?
	minCost = 95+145+75+105+90+10;
	for(unsigned int i=0; i<reaches->size(); i++)
	{
		temp = reaches->at(i);
		double cost = abs(temp->shoulderPitch) + abs(temp->shoulderRoll) +
						abs(temp->shoulderYaw) + abs(temp->elbow);
		if(min(cost, minCost)==cost)
		{
			armReach = temp;
			minCost = cost;
		}

	}
	*pCost = minCost;
	return armReach;
}


int ReachField::getLeftReachCount() const
{
	return leftReaches.size();
}
int ReachField::getRightReachCount() const
{
	return rightReaches.size();
}

int ReachField::getUsage() const
{
	return usage;
}

void ReachField::use()
{
	usage++;
}


double ReachField::getRadius() const
{
    return radius;
}

void ReachField::setRadius(double radius)
{
    this->radius = radius;
}


vector<ReachConfig*> ReachField::getLeftReaches() const
{
    return leftReaches;
}

vector<ReachConfig*> ReachField::getRightReaches() const
{
    return rightReaches;
}

bool ReachField::getPreferredArm() const
{
	if(leftReaches.size()>rightReaches.size())
		return false;
	else
		return true;
}


bool ReachField::isIntersecting() const
{
	return intersects;
}
void ReachField::setIntersects(bool overlap)
{
	intersects = overlap;
}



//non-member functions
ostream& operator <<(ostream& outs, const ReachField& rf)
{
	double x, y;
	rf.getReachDirection(&x,&y);
	outs << "X: " << x << ", Y: " << y << " ";
	outs << "reachConfigCount: " << (rf.getLeftReachCount() + rf.getRightReachCount());
	return outs;
}




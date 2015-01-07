/*
 * TorsoReach.cpp
 *
 *  Created on: 30 May 2012
 *      Author: icub
 */

#include "TorsoReach.h"

using namespace std;

using namespace yarp::os;

TorsoReach::TorsoReach(torsoSaccading* t, handEyeCoordination* h, EyeHeadSaccading* ehc)
{
	tor = t;
	heCoor = h;
	ehCont = ehc;
	gm = heCoor->getGazeMap();
	points();
}

TorsoReach::~TorsoReach()
{
}




bool TorsoReach::gazeReach(string colour, double range)
{
	double targX, targY, depth, gazeX, gazeY;

	heCoor->armRest();
	tor->compensateToRest();
	while(!heCoor->stationary())
	{
		Time::delay(0.5);
	}

	ehCont->getTarget()->getTarget(&targX, &targY, colour);
	bool success = ehCont->fixate(targX, targY, colour, true);
	if(!success)
		return false;

	ehCont->centreEyesInHead();
	ehCont->autoCenter(colour);
	Time::delay(1);
	ehCont->getDepth(colour, &depth);
	printf("Target is estimated at a depth of %.2fcm\n", depth);
	ehCont->getGazeDirection(&gazeX, &gazeY);
	GazeField* gf = ehCont->getGazeField();
	success = heCoor->hasLinkedReach(gf, depth);
	ReachField* rf;
	if(success)
	{
		rf = heCoor->getLinkedReach(gf, depth);
		cout << "Got linked reach: " << *rf << endl;
	}
	else
	{
		double dist;
		GazeReachLink* link;
		success = heCoor->getNearestLinkForGaze(gf, &link, depth, &dist);
		rf = link->reach;
		if(success)
		{
			cout << "Found a nearby reach field at a distance of " << dist << " away: " << *rf << endl;
			cout << "Gaze offset: " << (link->getGazeX()-gazeX) << ", " << (link->getGazeY()-gazeY) << endl;

			//May well be worth activating the torso to bring this into direct gaze line.
			tor->getTorsoController()->move(link->getGazeX()-gazeX, 0, true);
		}
		else
		{
			cout << "Couldn't even find a nearby reach, within a radius of however many units" << endl;
			//try searching with the torso
		}
	}

//	if(success)
//	{
		//Follow reach
		heCoor->reachTo(rf);
//	}
//	else
//	{
//		//consider using torso
//	}
//
	while(!heCoor->stationary())
	{
		Time::delay(1);
	}

	Time::delay(5);

	return success;


}









bool TorsoReach::getReach(double targetGazeX, double targetGazeY, double targetDepth, string colour, double range)
{
	//Lets start off by producing a list of possible matching reaches based on depth
	double closestDist=range*100;
	GazeReachLink* closestReachLink;

	vector<GazeReachLink*> possibles;

	vector<GazeReachLink*> links = gm->getLinks();
	for(size_t i=0; i<links.size(); i++)
	{
		GazeReachLink* l = links.at(i);
		double linkDepth = l->getGazeDepth();

		if(abs(linkDepth-targetDepth)<range && (linkDepth-targetDepth)<1)
		{
			possibles.push_back(l);
			cout << "Found a possible reach: " << *l << endl;

			if(abs(linkDepth-targetDepth)<closestDist)
			{
				closestDist = abs(linkDepth-targetDepth);
				closestReachLink = l;
			}


		}

	}

	printf("There are %i possible reaches to choose from.\n",possibles.size());

	if(closestDist!=range*100)
	{
		printf("Nearest reach was at a distance of %.2f away\n", closestDist);
		cout << *closestReachLink << endl;

//		//This approach while it can sometimes be really good, generally seems slightly less reliable
//		double gazeChange = targetGazeX-closestReachLink->getGazeX();
//		gazeChange = eyeXtoHeadX(gazeChange);
//
//		if(tor->isMotorField(-gazeChange,0))
//		{
//			PolarField* motor = tor->getMotorField(-gazeChange,0);
//			tor->gotoField(motor, true);
//		}
//		else if(abs(gazeChange)<4)
//		{
//			tor->getTorsoController()->move(-gazeChange,0,true);
//		}
//		else if(abs(gazeChange)>50)
//			return false;
//
//		ehCont->autoCenter(colour);
//		heCoor->reachTo(closestReachLink->reach);
//		while(!heCoor->stationary())
//			Time::delay(0.5);
//		return true;

	}

	//Now need to consider the gaze direction of the reach compared to the target gaze direction.
	// If there is a reach in the desired gaze direction, without any torso intervention, then this is probably the preferred reach.

	//Gaze map uses kinematics to supposedly calculate all eye directions, so need to convert eye back to torso motor values.
	double closestGaze=range*100;
	sort(possibles.begin(), possibles.end(), compareGazeLinks);		//sort links based on world coordinate height above table
	GazeReachLink* closestGazeLink;
	for(int i=0; i<possibles.size(); i++)
	{
		GazeReachLink* l = possibles.at(i);
		double diff = sqrt(pow(targetGazeX-l->getGazeX(),2)+pow(targetGazeY-l->getGazeY(),2));
		if(diff < closestGaze)
		{
			closestGaze = diff;
			closestGazeLink = l;
		}
	}


	if(closestGaze!=range*100)
	{
		printf("Nearest gaze was at a distance of %.2f away\n", closestGaze);
		cout << *closestGazeLink << endl;

		double gazeChange = targetGazeX-closestGazeLink->getGazeX();
		gazeChange = eyeXtoHeadX(gazeChange) * 1.25;

		if(tor->isMotorField(-gazeChange,0))
		{
			PolarField* motor = tor->getMotorField(-gazeChange,0);
			tor->gotoField(motor, true);
//			return true;
		}
		else if(abs(gazeChange)<4)
		{
			tor->getTorsoController()->move(-gazeChange,0,true);
		}
		else if(abs(gazeChange)>50)
			return false;

		ehCont->autoCenter(colour);
		heCoor->reachTo(closestGazeLink->reach);
		while(!heCoor->stationary())
			Time::delay(0.5);
		return true;

	}
	return false;

}

/**
 * Using the same formulae for generating the polar maps,
 * this generates just the points in a line across the map,
 * i.e. the coordinates for the fields in the torso motor map
 * that have been linked.  These points can be used to search for
 * matching reach links working from the centre out, alternating from
 * one side to the other.
 */
void TorsoReach::points()
{
	float minX,maxX,minY,maxY;
	minX=-101;
	maxX=101;
	minY=-61;
	maxY=61;
	float overlap = 1.4;
	int lines = 20;
	int rings = 40;
	float omega = 0.77;

	float proportionHorizontal = (abs(minX)+abs(maxX))/320.0;
	float proportionVertical = (abs(minY)+abs(maxY))/240.0;

	float proportion=0;
	if(proportionHorizontal>proportionVertical)
		proportion = proportionHorizontal;
	else
		proportion = proportionVertical;

	float radiusCompensation = 1.02*pow((double)proportion,-0.78);
	float centreX = (minX+maxX)/2;
	float angle, distance, c_x;

	for(int j=4; j<rings; j+=2)		//Even elements only
	{
		for(int i=0;i<lines;i+=(lines/2))	//Alternate sides, i.e. y=0
		{
			angle = i/lines*360;

			distance = rings*omega*j*pow(1.013,j)/10;
			distance = proportion*distance;

			c_x = centreX + distance*cos(toRadians(angle));

			if(c_x > minX && c_x < maxX)		//limit it to just the rings that fall within the motor range.
			{
				c_x = ((float)((int)(c_x*100)))/100;
				torsoPoints.push_back(c_x);
			}
		}
	}
}

/**
 *  x and y are the central points of motor fields in the torso map.
 */
bool TorsoReach::search(float x, float y)
{
	printf("Searching at (%.2f, %.2f)\n", x, y);

	PolarField* motor = tor->getMotorField(x,y);
	if(tor->isLinkedInput(motor))
	{
		PolarField* retina = tor->getLinkedInput(motor);
		printf("Gaze change from retina field is: (%.2f, %.2f)\n", retina->getXcoord(), retina->getYcoord());

	}

	float absolutePan = motor->getXcoord();

	vector<GazeReachLink*> links = gm->getLinks();
	for(size_t i=0; i<links.size(); i++)
	{
		GazeReachLink* l = links.at(i);

		if(abs(l->getGazeX() - absolutePan) < motor->getRadius())
		{
			cout << "gaze-arm link at: " << *l << " for torso at (" << x << ", " << y << ")" << endl;
		}

	}
}



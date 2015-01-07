/*
 * gazeField.cpp
 *
 *  Created on: 30 Mar 2011
 *      Author: icub
 */

#include "GazeField.h"
using namespace std;

ostream& operator <<(ostream& outs, const HeadConfig& source)
{
	outs << "neck: [" <<source.headX() << "," << source.headY() << "] ";
	outs << "eye: [" << source.eyeX() << "," << source.eyeY() << "]";
	return outs;
}

bool operator ==(const HeadConfig& h1, const HeadConfig& h2)
{
	return h1.eyeTilt==h2.eyeTilt && h1.eyeVersion==h2.eyeVersion &&
			h1.neckPitch==h2.neckPitch && h1.neckYaw==h2.neckYaw;
}



GazeField::GazeField()
{
	gazed = false;
//    gazePosCount = 0;

	x = 0;
	y = 0;
	index = 0;
	eyeHeadPositions.reserve(5);
	usage=0;

	wx=wy=wz=vergence=0;

	radius = 3;
}

GazeField::GazeField(size_t fieldID, double gazeX, double gazeY, double* eyeHeadMotor, double rad)
{
	gazed = false;
//    gazePosCount = 0;

	x = (float)gazeX;
	y = (float)gazeY;
	eyeHeadPositions.reserve(5);
	addGazeConfiguration(eyeHeadMotor);
	index = fieldID;
	usage=1;
	radius = rad;
	wx=wy=wz=vergence=0;
}

GazeField::GazeField(size_t fieldID, double gazeX, double gazeY, double worldX, double worldY, double worldZ, double pVerg, double* eyeHeadMotor, double rad)
{
	gazed = false;
//    gazePosCount = 0;

	x = (float)gazeX;
	y = (float)gazeY;
	eyeHeadPositions.reserve(5);
	addGazeConfiguration(eyeHeadMotor);
	index = fieldID;
	usage=1;
	radius = rad;
	wx=worldX;
	wy=worldY;
	wz=worldZ;
	vergence=pVerg;
}

GazeField::~GazeField()
{
	eyeHeadPositions.clear();
	eyeHeadPositions.resize(1);
}

void GazeField::getGazeDirection(double* gazeX, double* gazeY) const
{
	*gazeX = (double)x;
	*gazeY = (double)y;
}



void GazeField::addGazeConfiguration(double* gaze)
{
//	cout << eyeHeadPositions.size() << endl;
	HeadConfig* hc = new HeadConfig(gaze);
	for(int i=0; i<eyeHeadPositions.size(); i++)
	{
		HeadConfig* temp = eyeHeadPositions.at(i);
		if(*hc == *temp)
		{
			return;
		}
	}
	//No matching existing pose;
	eyeHeadPositions.push_back(new HeadConfig(gaze));

}




HeadConfig* GazeField::getPreferredGaze()
{	//should aim for a gaze where the eyes are close to central
	double minX, minY, minEye=100;
	HeadConfig* temp = new HeadConfig();
	minX = 135;
	minY = 145;
	HeadConfig* motorPos = new HeadConfig();
//	cout << "gazePosCount: " << gazePosCount << endl;
//	cout << "gazeList size: " << eyeHeadPositions.size() << endl;
	for(int i=0; i<eyeHeadPositions.size(); i++)
	{
		temp = eyeHeadPositions.at(i);
//		cout << *temp << endl;
		double hX, hY, eX, eY;
		double costX, costY;
		hX = temp->headX();
		hY = temp->headY();
		eX = temp->eyeX();
		eY = temp->eyeY();

		costX = abs(hX) + abs(eX);
		costY = abs(hY) + abs(eY);

		double costEye = abs(eX) + abs(eY);

		if(min(minEye,costEye)==costEye)
		{
			motorPos = eyeHeadPositions.at(i);
			minEye = costEye;
		}
//		if(min(minX,costX)==costX && min(minY,costY)==costY)
//		{
//			motorPos = eyeHeadPositions.at(i);
//			minX = costX;
//			minY = costY;
//		}
	}
	return motorPos;
}



int GazeField::getGazeCount() const
{
//	return gazePosCount;
	return eyeHeadPositions.size();
}

int GazeField::getUsage() const
{
	return usage;
}

void GazeField::use()
{
	usage++;
}


double GazeField::getRadius() const
{
    return radius;
}

void GazeField::setRadius(double radius)
{
    this->radius = radius;
}



string GazeField::getComments() const
{
    return comments;
}

vector<HeadConfig*> GazeField::getEyeHeadPositions() const
{
    return eyeHeadPositions;
}



void GazeField::setComments(string comments)
{
    this->comments = comments;
}





/////////////////////////// Private functions  ///////////////////////////////




//non-member functions
ostream& operator <<(ostream& outs, const GazeField& gf)
{
	double x, y;
	gf.getGazeDirection(&x,&y);
	outs << "gaze: [" << x << "," << y << "] gazeConfigCount: " << gf.getGazeCount();
	return outs;
}

bool operator ==(const GazeField& gf1, const GazeField& gf2)
{
	return gf1.x == gf2.x && gf1.y==gf2.y;
}
bool operator <(const GazeField& gf1, const GazeField& gf2)
{
	return gf1.index < gf2.index;
}


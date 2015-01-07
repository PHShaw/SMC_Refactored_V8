/*
 * GazeMap.cpp
 *
 *  Created on: 30 Mar 2011
 *      Author: icub
 */

#include "GazeMap.h"
using namespace std;


bool operator ==(const GazeReachLink& l1, const GazeReachLink& l2)
{
	return l1.gaze ==l2.gaze  &&
		   l1.reach==l2.reach &&
		   l1.gazeDepth==l2.gazeDepth;
}

ostream& operator <<(ostream& outs, const GazeReachLink& source)
{
	outs << "Gaze-Arm link at: Gaze(" << source.getGazeX() << ", " << source.getGazeY() <<
							 ") Arm(" << source.getArmFlexion() << ", " << source.getArmAngle() <<
							 ") Depth(" << source.getGazeDepth() << ") ";
	return outs;
}

bool compareGazeLinks(GazeReachLink* l1, GazeReachLink* l2)
{
	if(l1->getArmAngle() < l2->getArmAngle())
		return true;
	else
		return false;

//	if(l1->gazeDepth < l2->gazeDepth)
//		return true;
//	else
//		return false;
}


GazeMap::GazeMap()
{
//	numFields = 0;
	fieldIDCounter = 0;
	gazeFields.reserve(10);
	reachFields.reserve(10);

}

GazeMap::~GazeMap()
{
	gazeFields.clear();
	gazeFields.resize(1);
	reachFields.clear();
	reachFields.resize(1);
}


void GazeMap::addGazeField(double* headMotorConfig)
{
	double gazeX, gazeY, gazeZ;

	//eye head not quite 1:1 so using the eye-head compensation
	//formulae here to produce a more accurate gaze direction in eye scale
//	gazeX = headMotorConfig[eyeX] - headXtoEyeX(headMotorConfig[headX]);
//	gazeY = headMotorConfig[eyeY] - headYtoEyeY(headMotorConfig[headY]);

//	double torso[3] = {0,0,0};
//	double head[6] = {0,0,0,0,0,0};
//	CalculateGazeChange(torso, head, torso, headMotorConfig, &gazeY, &gazeX, &gazeZ);

	gazeX = headMotorConfig[eyeX] - headXtoEyeX(headMotorConfig[headX]);
	gazeY = headMotorConfig[eyeY] - headYtoEyeY(headMotorConfig[headY]);

//	cout << "*********** Gaze: " << gazeX << " " << gazeY << "************" << endl;

	addGazeField(gazeX, gazeY, headMotorConfig);

}

void GazeMap::addGazeField(double gazeX, double gazeY, double* headMotorConfig, double radius)
{
	//look for nearest existing gaze field.
	double dist;
	double torsoMotorConfig[3] = {0,0,0};
	double x,y,z;
	CalculateTargetWorldRef(torsoMotorConfig, headMotorConfig, &x, &y, &z);	// returned in mm

	GazeField* gf = new GazeField();
	getNearestGazeField(gazeX, gazeY, &gf, &dist, radius);
	if(dist < 0.2)
	{
		gf->addGazeConfiguration(headMotorConfig);
		if(headMotorConfig[5]>=1)
		{
			if(gf->wx==0 && gf->wy==0 &&gf->wz==0)
			{
				gf->wx = x;
				gf->wy = y;
				gf->wz = z;
				gf->vergence = headMotorConfig[5];
			}
		}
	}
	else
	{
		if(headMotorConfig[5]>=1)
		{
			gazeFields.push_back(new GazeField(fieldIDCounter, gazeX, gazeY, x,y,z,headMotorConfig[5], headMotorConfig, radius));
		}
		else
		{
			gazeFields.push_back(new GazeField(fieldIDCounter, gazeX, gazeY, 0,0,0,0, headMotorConfig, radius));
		}
		fieldIDCounter ++;
	}


//	GazeField* gf = getGazeField(gazeX,gazeY);
//	double x,y;
//	gf->getGazeDirection(&x, &y);
//	if (!(x==0 && y==0))
//	{
//		gf->addGazeConfiguration(headMotorConfig);
//	}
//	else
//	{
//		gazeFields.push_back(new GazeField(fieldIDCounter, gazeX, gazeY, headMotorConfig, radius));
//		fieldIDCounter ++;
//	}
}

void GazeMap::addGazeField(double gazeX, double gazeY, double worldX, double worldY, double worldZ, double vergence, double* headMotorConfig, double radius)
{
	//look for nearest existing gaze field.
	double dist;
	GazeField* gf = new GazeField();
	getNearestGazeField(gazeX, gazeY, &gf, &dist, radius);
	if(dist < 0.3)
	{
		gf->addGazeConfiguration(headMotorConfig);
	}
	else
	{
		gazeFields.push_back(new GazeField(fieldIDCounter, gazeX, gazeY, worldX, worldY, worldZ, vergence, headMotorConfig, radius));
		fieldIDCounter ++;
	}

}


void GazeMap::addReachField(double* armReachConfig, bool rightArm, double radius)
{
	//look for nearest reach field.
	double dist;
	ReachField* rf = new ReachField();
	ReachConfig* rc = new ReachConfig(armReachConfig, rightArm);
	double depth = rc->flexion();
	double angle = rc->rotation();
	getNearestReachField(depth, angle, &rf, &dist, radius);
	if(dist < 0.2)
	{
		rf->addReachConfiguration(armReachConfig, rightArm);
		return;
	}
	else
	{
		reachFields.push_back(new ReachField(reachFields.size(),depth, angle, armReachConfig, rightArm,radius));
	}

//	ReachField* rf = getReachField(armReachConfig, rightArm);
//	double x,y;
//	rf->getReachDirection(&x, &y);
//	if(!(x==0 && y==0))
//	{
//		rf->addReachConfiguration(armReachConfig, rightArm);
//		return;
//	}
//	else
//	{
//		ReachConfig* rc = new ReachConfig(armReachConfig, rightArm);
//
//		double angle  = rc->rotation();
////		if(!rightArm)
////			angle *= -1;
//		reachFields.push_back(new ReachField(reachFields.size(),rc->flexion(), angle, armReachConfig, rightArm,radius));
//	}
}


//Add gaze field if necessary, add reach field if necessary, then link the two together.
void GazeMap::addGazeReach(double* headMotorConfig, double* armReachConfig, bool rightArm, double depth)
{
	addGazeField(headMotorConfig);		//will add if doesn't already exist
	addReachField(armReachConfig, rightArm);	//will add if doesn't already exist

	GazeField* gf = getGazeField(headMotorConfig);
	ReachField* rf = getReachField(armReachConfig, rightArm);

	addCoordination(gf, rf, depth);


//
//
//	double gazeX, gazeY;
//
//	//eye head not quite 1:1 so using the eye-head compensation
//	//formulae here to produce a more accurate gaze direction in eye scale
//	gazeX = (int)(headMotorConfig[eyeX] - headXtoEyeX(headMotorConfig[headX]));
//	gazeY = (int)(headMotorConfig[eyeY] - headYtoEyeY(headMotorConfig[headY]));
//
//	GazeField* gf = getGazeField(gazeX,gazeY);
//	double x,y;
//	gf->getGazeDirection(&x, &y);
//	if (!(x==0 && y==0))
//	{
//		gf->addGazeConfiguration(headMotorConfig);
//		gf->addReachConfiguration(armReachConfig, rightArm);
//	}
//	else
//	{
//		gf = new GazeField(fieldIDCounter, gazeX, gazeY, headMotorConfig);
//		gf->addReachConfiguration(armReachConfig, rightArm);
//		fieldIDCounter ++;
//		numFields++;
//		gazeFields.push_back(gf);
//	}
}


//Attempts to return at field at precise coordinates;
GazeField* GazeMap::getGazeField(double gazeX, double gazeY)
{
	GazeField* temp;
	GazeField* closest;
	double minDist=10000;
//	cout << "Num fields: " << gazeFields.size() << endl;
//	cout << "********************* Looking for: " << (int)gazeX << " " << (int)gazeY << "**********" << endl;
	for(unsigned int i=0; i<gazeFields.size(); i++)
	{
		temp = gazeFields.at(i);
//		cout << "****************************" << (int)(temp->getXcoord()) << "*********" << (int)(temp->getYcoord())<< "*******************" << endl;
//		cout << *temp << endl;


		double dist, gx, gy;
		gx = temp->getXcoord();
		gy = temp->getYcoord();
		dist = sqrt(pow(gx-gazeX,2) + pow(gy-gazeY,2));

		if(dist < temp->getRadius())	//check if point falls within field radius
		{
			if(dist<minDist)	//check to see if point is less than previously found points
			{
				minDist = dist;
				closest = temp;
			}
		}


//		if((int)(temp->getXcoord())==(int)gazeX && (int)(temp->getYcoord())==(int)gazeY)
//		{
////			cout << "******************* FOUND *******************" << endl;
//			return gazeFields.at(i);
//
//		}
	}

	if(minDist!=10000)
	{
		return closest;
	}
	else
	{
//		cout << "******************* GAZE FIELD NOT FOUND :( *******************" << endl;
		return new GazeField();
	}
}

GazeField*  GazeMap::getGazeField(double worldX, double worldY, double worldZ)
{
	GazeField* temp;
	GazeField* closest;
	double minDist=10000;
//	cout << "Num fields: " << gazeFields.size() << endl;
//	cout << "********************* Looking for: " << (int)gazeX << " " << (int)gazeY << "**********" << endl;
	for(unsigned int i=0; i<gazeFields.size(); i++)
	{
		temp = gazeFields.at(i);
//		cout << "****************************" << (int)(temp->getXcoord()) << "*********" << (int)(temp->getYcoord())<< "*******************" << endl;
//		cout << *temp << endl;

		if(temp->wx==0 && temp->wy==0 && temp->wz==0)
			continue;

		double dist;
		dist = sqrt(pow(temp->wx-worldX,2) + pow(temp->wy-worldY,2)+ pow(temp->wz-worldZ,2));

		if(dist < temp->getRadius())	//check if point falls within field radius
		{
			if(dist<minDist)	//check to see if point is less than previously found points
			{
				minDist = dist;
				closest = temp;
			}
		}


//		if((int)(temp->getXcoord())==(int)gazeX && (int)(temp->getYcoord())==(int)gazeY)
//		{
////			cout << "******************* FOUND *******************" << endl;
//			return gazeFields.at(i);
//
//		}
	}

	if(minDist!=10000)
	{
		return closest;
	}
	else
	{
//		cout << "******************* GAZE FIELD NOT FOUND :( *******************" << endl;
		return new GazeField();
	}
}

bool GazeMap::getNearestGazeField(double worldX, double worldY, double worldZ, GazeField** field, double *distance,
								double radius)
{
	GazeField* temp;
	double closest = 10000;
	for(unsigned int i=0; i<gazeFields.size(); i++)
	{
		temp = gazeFields.at(i);
		if(temp->wx==0 && temp->wy==0 && temp->wz==0)
			continue;
		double hyp;
		hyp = sqrt(pow(temp->wx-worldX,2) + pow(temp->wy-worldY,2)+ pow(temp->wz-worldZ,2));
		if(hyp < closest)
		{
			closest = hyp;
			*field = temp;
		}
	}

	*distance = closest;
	if(closest >= radius)
	{
		return false;
	}
	else
	{
		return true;
	}
}


GazeField* GazeMap::getGazeField(double* headMotorConfig)
{
	double gazeX, gazeY, gazeZ;

//	double torso[3] = {0,0,0};
//	double head[6] = {0,0,0,0,0,0};
//	CalculateGazeChange(torso, head, torso, headMotorConfig, &gazeY, &gazeX, &gazeZ);
	printf("HMC: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]\n",headMotorConfig[0],headMotorConfig[1],headMotorConfig[2],headMotorConfig[3],headMotorConfig[4],headMotorConfig[5]);
	gazeX = headMotorConfig[eyeX] - headXtoEyeX(headMotorConfig[headX]);
	gazeY = headMotorConfig[eyeY] - headYtoEyeY(headMotorConfig[headY]);

	cout << "GazeX: " << gazeX << ", GazeY: " << gazeY << endl;

	return getGazeField(gazeX, gazeY);
}

ReachField* GazeMap::getReachField(double X, double Y)
{
	ReachField* temp;
	ReachField* closest;
	double minDist=10000;

	for(unsigned int i=0; i<reachFields.size(); i++)
	{
		temp = reachFields.at(i);

		double dist, rx, ry;
		rx = temp->getXcoord();
		ry = temp->getYcoord();
		dist = sqrt(pow(rx-X,2) + pow(ry-Y,2));

		if(dist < temp->getRadius())
		{
			if(dist<minDist)
			{
				minDist = dist;
				closest = temp;
			}
		}

//		if((int)(temp->getXcoord())==(int)depth && (int)(temp->getYcoord())==(int)angle)
//		{
//
//			return reachFields.at(i);
//		}
	}
	if(minDist!=10000)
	{
		return closest;
	}
	else
	{
		return new ReachField();
	}
}

ReachField* GazeMap::getReachField(double* armReachConfig, bool rightArm)
{
	ReachConfig* rc = new ReachConfig(armReachConfig, rightArm);
	double depth = rc->flexion();
	double angle = rc->rotation();

//	cout << "****************" << armReachConfig[0] << "*********" << armReachConfig[1] << "**********" << armReachConfig[2] << "************" << endl;
//	cout << "****************************" << depth << "*********" << angle << "*******************" << endl;

//	if(!rightArm)
//		angle *= -1;

	return getReachField(depth, angle);
}



bool GazeMap::getNearestGazeField(double gazeX, double gazeY, GazeField** field, double *distance,
								double radius)
{
	GazeField* temp;
	double closest = 10000;
	for(unsigned int i=0; i<gazeFields.size(); i++)
	{
		temp = gazeFields.at(i);
		double pX, pY;
		temp->getGazeDirection(&pX, &pY);
		double hyp;
		hyp = sqrt((gazeX-pX)*(gazeX-pX) + (gazeY-pY)*(gazeY-pY));
		if(hyp < closest)
		{
			closest = hyp;
			*field = temp;
		}
	}

	*distance = closest;
	if(closest >= radius)
	{
		return false;
	}
	else
	{
		return true;
	}
}


bool GazeMap::getNearestReachField(double flexion, double angle, ReachField** field, double *distance,
								double radius)
{
	ReachField* temp;
	double closest = 10000;
	for(unsigned int i=0; i<reachFields.size(); i++)
	{
		temp = reachFields.at(i);
		double pD, pA;
		temp->getReachDirection(&pD, &pA);
		double hyp;
		hyp = sqrt((flexion-pD)*(flexion-pD) + (angle-pA)*(angle-pA));
		if(hyp < closest)
		{
			closest = hyp;
			*field = temp;
		}
	}

	*distance = closest;
	if(closest >= radius)
		return false;
	else
	{
		return true;
	}
}

// simple calculates the depth and angle for a given arm configuration
void GazeMap::armData(double* armReach, bool rightArm, double* depth, double* angle)
{
	ReachConfig* rc = new ReachConfig(armReach, rightArm);
	*depth = rc->flexion();
	*angle = rc->rotation();
}

void GazeMap::gazeData(double* headMotorConfig, double* gazeX, double* gazeY)
{
	cout <<"GM-HeadMotorConfig: ";
	for(int i=0; i<6; i++)
	{
		cout << headMotorConfig[i] << ", ";
	}
	cout << endl;

	*gazeX = headMotorConfig[eyeX] - headXtoEyeX(headMotorConfig[headX]);
	*gazeY = headMotorConfig[eyeY] - headYtoEyeY(headMotorConfig[headY]);

//	double gazeZ;
//	double torso[3] = {0,0,0};
//	double head[6] = {0,0,0,0,0,0};
//	CalculateGazeChange(torso, head, torso, headMotorConfig, gazeY, gazeX, &gazeZ);

}



	//rewritten this to look for the nearest reach link using depth and angle...
  //Then add the inverse to find the nearest armreach for gaze
bool GazeMap::getNearestGazeFieldForArmReach(double* armReach, bool isRightArm, GazeField** field,
											double* correlation, double radius)
{
	double flexion, angle;
	armData(armReach, isRightArm, &flexion, &angle);
	double minCorr = radius;

	double closestCorr = 20000;
	GazeReachLink* closestLink = new GazeReachLink();
	for(size_t i=0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* l = handEyeCoordination.at(i);
		double flex, ang;
		flex = l->getArmFlexion();
		ang = l->getArmAngle();

		double corr;
		corr = sqrt((flexion - flex)*(flexion - flex) + (angle - ang)*(angle - ang));
		if(corr < minCorr)
		{
			minCorr = corr;
			closestCorr = corr;
			closestLink = l;
		}
		else if(corr < closestCorr)
		{
			closestCorr = corr;
			closestLink = l;
		}
	}
	if(minCorr != radius)
	{
		*field = closestLink->gaze;
		*correlation = minCorr;
		return true;
	}
	else if(closestCorr != 20000) //found something, but it wasn't very close
	{
		*field = closestLink->gaze;
		*correlation = closestCorr;
		return false;
	}
	else	//Didn't find anything
	{
		*field = new GazeField();
		return false;
	}

}


bool GazeMap::getNearestReachFieldForGaze(double* headMotorConfig, double depth, ReachField** field,
											double* correlation, double radius)
{
	double gazeX, gazeY;
	gazeData(headMotorConfig, &gazeX, &gazeY);
	double minCorr = radius;

	double closestCorr = 20000;
	GazeReachLink* closestLink = new GazeReachLink();
	for(size_t i=0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* l = handEyeCoordination.at(i);
		double gX, gY;
		l->gaze->getGazeDirection(&gX,&gY);
		double corr;
		corr = sqrt((gazeX - gX)*(gazeX - gX) + (gazeY - gY)*(gazeY - gY) +
					(depth-l->gazeDepth)*(depth-l->gazeDepth));
		if(corr < minCorr)
		{
			minCorr = corr;
			closestCorr = corr;
			closestLink = l;
		}
		else if(corr < closestCorr)
		{
			closestCorr = corr;
			closestLink = l;
		}
	}
	if(minCorr != radius)
	{
		*field = closestLink->reach;
		*correlation = minCorr;
		return true;
	}
	else if(closestCorr != 20000) //found something, but it wasn't very close
	{
		*field = closestLink->reach;
		*correlation = closestCorr;
		return false;
	}
	else	//Didn't find anything
	{
		*field = new ReachField();
		return false;
	}
}


bool GazeMap::getNearestReachFieldForGaze(double* headMotorConfig, ReachField** field,
											double* correlation, double radius)
{
	double gazeX, gazeY;
	gazeData(headMotorConfig, &gazeX, &gazeY);
	cout << "GazeX " << gazeX << ", GazeY " << gazeY << endl;
	double minCorr = radius;

	double closestCorr = 20000;
	GazeReachLink* closestLink = new GazeReachLink();
	for(size_t i=0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* l = handEyeCoordination.at(i);
		cout << i << ". " << *l << endl;
		double gX, gY;
		l->gaze->getGazeDirection(&gX,&gY);
		double corr;
		corr = sqrt((gazeX - gX)*(gazeX - gX) + (gazeY - gY)*(gazeY - gY));

		if(corr < closestCorr)
		{
			closestCorr = corr;
			closestLink = l;
		}

		if(corr < minCorr)
		{
			minCorr = corr;
			closestCorr = corr;
			closestLink = l;
		}


	}
	if(minCorr != radius)
	{
		*field = closestLink->reach;
		*correlation = minCorr;
		return true;
	}
	else if(closestCorr != 20000) //found something, but it wasn't very close
	{
		*field = closestLink->reach;
		*correlation = closestCorr;
		return false;
	}
	else	//Didn't find anything
	{
		*field = new ReachField();
		return false;
	}
}



vector<GazeReachLink*> GazeMap::getReachSetForGaze(double* headMotorConfig, double radius)
{
	vector<GazeReachLink*> fields;
	double gazeX, gazeY;
	gazeData(headMotorConfig, &gazeX, &gazeY);
	for(size_t i=0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* l = handEyeCoordination.at(i);
		double gX, gY;
		l->gaze->getGazeDirection(&gX,&gY);
		double corr;
		corr = sqrt((gazeX - gX)*(gazeX - gX) + (gazeY - gY)*(gazeY - gY));
		if(corr < radius)
		{
			fields.push_back(l);
		}
	}
	return fields;
}


GazeReachLink* GazeMap::getLinkFromReach(ReachField* rf)
{
	for(size_t i=0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* l = handEyeCoordination.at(i);
		if(l->reach == rf)
			return l;
	}
	return 0;
}

/**
 * Returns false if the link already exists, otherwise true
 */
bool GazeMap::addCoordination(double gazeX, double gazeY, double flexion, double angle, double depth)
{
	GazeField* gf = getGazeField(gazeX, gazeY);
	ReachField* rf = getReachField(flexion, angle);
	return addCoordination(gf, rf, depth);


//	//first check to see if such a link exists already.
//	GazeReachLink* l = new GazeReachLink(gazeX, gazeY, depth, angle);
//
//	for(size_t i = 0; i<handEyeCoordination.size(); i++)
//	{
//		GazeReachLink* tmp = handEyeCoordination.at(i);
//		if(l == tmp)
//			return false;
//	}
//	//no matching link found
//	handEyeCoordination.push_back(l);
//	return true;

}


bool GazeMap::updateLink(ReachField* rf, GazeField* newGF, double depth)
{
	GazeReachLink* l = getLinkFromReach(rf);
	if(l!=0)
	{
		l->gaze = newGF;
		l->gazeDepth = depth;
		return true;
	}
	else
		return false;
}

bool GazeMap::addCoordination(GazeField* gf, ReachField* rf, double depth)
{
	GazeReachLink* l = new GazeReachLink(gf, rf,depth);

	for(size_t i = 0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* tmp = handEyeCoordination.at(i);
		if(l == tmp)
			return false;
	}
	//no matching link found
	handEyeCoordination.push_back(l);
	return true;



//	double gazeX, gazeY, depth, angle;
//	gf->getGazeDirection(&gazeX, &gazeY);
//	rf->getReachDirection(&depth, &angle);
//	return addCoordination(gazeX, gazeY, depth, angle);
}


bool GazeMap::getHandCoordination(GazeField* gf, ReachField** rf, double depth)
{
	for(size_t i = 0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* tmp = handEyeCoordination.at(i);
		if(tmp->gaze==gf && tmp->gazeDepth==depth)
		{
			*rf = tmp->reach;
			return true;
		}
	}
	*rf = new ReachField();
	return false;
}

bool GazeMap::getHandCoordination(GazeField* gf, ReachField** rf)
{
	for(size_t i = 0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* tmp = handEyeCoordination.at(i);
		if(tmp->gaze==gf)
		{
			*rf = tmp->reach;
			return true;
		}
	}
	*rf = new ReachField();
	return false;
}


bool GazeMap::getEyeCoordination(ReachField* rf, GazeField** gf)
{
	for(size_t i = 0; i<handEyeCoordination.size(); i++)
	{
		GazeReachLink* tmp = handEyeCoordination.at(i);
		if(tmp->reach==rf)
		{
			*gf = tmp->gaze;
			return true;
		}
	}
	*gf = new GazeField();
	return false;
}





bool GazeMap::matchArmReach(ReachConfig* arm1, ReachConfig* arm2, double* comparison, double radius)
{
	double sp, sr, sy, e;
	sp = abs(arm1->shoulderPitch-arm2->shoulderPitch);
	sr = abs(arm1->shoulderRoll-arm2->shoulderRoll);
	sy = abs(arm1->shoulderYaw-arm2->shoulderYaw);
	 e = abs(arm1->elbow-arm2->elbow);

	*comparison = sp + sr + sy + e;

	if(sp<radius && sr<radius && sy<radius && e<radius)
		return true;
	else
		return false;

}


double headXtoEyeX(double headX)
{
	return (headX + 0.3) / 0.86;
}
double headYtoEyeY(double headY)
{
	return (headY - 0.28) / -0.87;
}
double eyeXtoHeadX(double eyeX)
{
	return eyeX * 0.86 - 0.3;
}
double eyeYtoHeadY(double eyeY)
{
	return eyeY * -0.87 + 0.28;
}

vector<GazeField*> GazeMap::getGazeFields() const
{
    return gazeFields;
}

vector<ReachField*> GazeMap::getReachFields() const
{
    return reachFields;
}

vector<GazeReachLink*> GazeMap::getLinks() const
{
	return handEyeCoordination;
}

unsigned int GazeMap::getNumGazeFields() const
{
    return gazeFields.size();
}


unsigned int GazeMap::getNumReachFields() const
{
    return reachFields.size();
}

unsigned int GazeMap::getNumLinks() const
{
    return handEyeCoordination.size();
}


/*
 * target.cpp
 *
 *  Created on: 10 Jan 2011
 *      Author: icub
 */
#ifndef TARGET_CPP
	#define TARGET_CPP
#include "Target.h"

using namespace yarp::os;
using namespace yarp::dev;

using namespace std;



Target::Target()
{
	ulster = false;
	initTarget("icub");
	visible = false;
}
Target::Target(string robot)
{
	ulster = false;
	initTarget(robot);
	visible = false;
}

Target::Target(bool uu)
{
	ulster = uu;
	initTarget("icub");
	if(uu){
		requestROI =true;
		sentFixate = false;
	}

	visible = false;
}


bool Target::targetVisible()
{
	return visible;
}

bool Target::initTarget(string robot)
{
	Network yarp;

	if(ulster)
	{
		portUU.open("/target/status");
//		yarp.connect("/rosyarp/saccade_coords", "/target/read");
//		yarp.connect("/target/status", "/rosyarp/saccade_command");

	}
	else
	{
		if(robot.compare("icubSim")==0)
		{
			porttargetsleft.open("/targetSim/left/read");
			porttargetsright.open("/targetSim/right/read");

			yarp.connect("/targetSim/left/data", "/targetSim/left/read");
			yarp.connect("/targetSim/right/data", "/targetSim/right/read");
		}
		else
		{
			porttargetsleft.open("/target/left/read");
			porttargetsright.open("/target/right/read");

			yarp.connect("/target/left/data", "/target/left/read");
			yarp.connect("/target/right/data", "/target/right/read");
		}

	}


	centred = false;
}

bool Target::getTarget(double* targX, double* targY)
{
	if(ulster)
		return uuGetTarget(targX, targY);

//	porttargetsright.read(0);	//Clears last target for when target is no longer visible
	Time::delay(0.15);
	if (target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numColours = size/elements;
			int col = randGenerator(numColours);
			printf("***********Target colours detected: %i, Target selected: %i*************\n", numColours, col);

			col *= elements;

			visible = true;
			colour = target->get(0+col).asString();
			*targX = target->get(1+col).asDouble();
			*targY = target->get(2+col).asDouble();
			size   = target->get(3+col).asInt();
			printf("Colour: %s  X: %.3f  Y: %.3f\n",
					colour.c_str(),
					*targX, *targY);

			targetlogfile << target->get(0).asString().c_str() << " " <<
					*targX << " " << *targY;

			double dist;
			if(fovea(*targX, *targY, &dist))
			{
				printf("Central\n");
				targetlogfile << " " << dist << " Central";
			}
			else
				targetlogfile << " " << dist;

			targetlogfile << endl;
			lastX = *targX;
			lastY = *targY;

			return true;
		}
		else
		{
			visible = false;
			*targX = 0;
			*targY = 0;
			printf("no target\n");
			targetlogfile << "- 0 0 No target" << endl;
			return false;
		}

	}
	else
	{
		visible = false;
		*targX = 0;
		*targY = 0;
		printf("no target\n");
		targetlogfile << "- 0 0 No target" << endl;
		return false;
	}
}

bool Target::getTarget(double* targX, double* targY, const string colourTest)	//look for specific colour
{
	if(ulster)
		return uuGetTarget(targX, targY);

//	porttargetsright.read(0);	//Clears last target for when target is no longer visible
	Time::delay(0.15);
	bool found = false;
	if(target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numColours = size/elements;
			for(int i=0; i<numColours; i++)
			{
				string col;
				col = target->get(i*elements).asString();
				if(col.compare(colourTest)==0)
				{
					visible = true;
					colour = target->get(i*elements).asString();
					*targX = target->get(i*elements +1).asDouble();
					*targY = target->get(i*elements +2).asDouble();
					size   = target->get(i*elements +3).asInt();
					printf("Colour: %s  X: %.3f  Y: %.3f\n",
							colour.c_str(),
							*targX, *targY);

					targetlogfile << colour << " " <<
							*targX << " " << *targY;

					double dist;
					if(fovea(*targX, *targY, &dist))
					{
						printf("Central\n");
						targetlogfile << " " << dist <<" Central";
					}
					else
						targetlogfile << " " << dist;
					targetlogfile << endl;
					lastX = *targX;
					lastY = *targY;
					return true;
				}
			}
		}
	}
	visible = false;
	*targX = 0;
	*targY = 0;
	printf("no target\n");
	targetlogfile << "- 0 0 No target of specified colour" << endl;
	return false;

}

Bottle* Target::getAllTargets()
{
	if(target = porttargetsright.read(0))
	{
		return target;
	}
	else
	{
		return new Bottle();
	}
}


string Target::getColour()
{
	return colour;
}

bool Target::matchColour(const string colourTest)
{
	if(colour.compare(colourTest)==0)
		return true;
	else
		return false;
}

string Target::getNearestObject(double* targX, double* targY, double* dist)
{
	Time::delay(0.15);
	bool found = false;
	string closestCol;
	double closestDist=3000, closestX, closestY;
	if(target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numColours = size/elements;
			for(int i=0; i<numColours; i++)
			{
				visible = true;
				string col = target->get(i*elements).asString().c_str();
				*targX = target->get(i*elements +1).asDouble();
				*targY = target->get(i*elements +2).asDouble();
				size   = target->get(i*elements +3).asInt();

				fovea(*targX, *targY, dist);
				if(*dist < closestDist)
				{
					closestCol = col;
					closestX = *targX;
					closestY = *targY;
					closestDist = *dist;
				}
			}
			*targX = closestX;
			*targY = closestY;
			*dist = closestDist;
			colour = closestCol;
			lastX = *targX;
			lastY = *targY;
			return closestCol;
		}
		return "";
	}
	visible = false;
	*targX = 0;
	*targY = 0;
}




vector<string> Target::getNearestObjects()
{
	vector<string> coloursDist;
	double targX, targY, dist;
	Time::delay(0.15);
	bool found = false;

	if(target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numColours = size/elements;
			double distances[numColours];

			for(int i=0; i<numColours; i++)
			{
				visible = true;
				string col = target->get(i*elements).asString().c_str();
				targX = target->get(i*elements +1).asDouble();
				targY = target->get(i*elements +2).asDouble();
				size   = target->get(i*elements +3).asInt();

				fovea(targX, targY, &dist);
				distances[i] = dist;
			}

			//Got the list of distances for each colour, now to sort colours based on that order

			for(int i=0; i<numColours; i++)
			{
				double minDist = 5000;
				int id;
				for(int j=0; j<numColours; j++)
				{
					if(distances[j]<minDist)
					{
						id = j;
						minDist = distances[j];
					}
				}
				coloursDist.push_back(target->get(id*elements).asString().c_str());
				distances[id] = 10000;

			}
			return coloursDist;

		}
	}
	return coloursDist;
}



std::string Target::getNearestTo(const int x, const int y, double* targX, double* targY, double* dist)
{
	Time::delay(0.15);
	bool found = false;
	string closestCol;
	double closestDist=3000, closestX, closestY;
	if(target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numColours = size/elements;
			for(int i=0; i<numColours; i++)
			{
				visible = true;
				string col = target->get(i*elements).asString().c_str();
				*targX = target->get(i*elements +1).asDouble();
				*targY = target->get(i*elements +2).asDouble();
				size   = target->get(i*elements +3).asInt();

				*dist = sqrt((*targX - x)*(*targX - x) + (*targY - y)*(*targY - y));
				if(*dist < closestDist)
				{
					closestCol = col;
					closestX = *targX;
					closestY = *targY;
					closestDist = *dist;
				}
			}
			*targX = closestX;
			*targY = closestY;
			*dist = closestDist;
			colour = closestCol;
			lastX = *targX;
			lastY = *targY;
			return closestCol;
		}
		return "";
	}
	visible = false;
	*targX = 0;
	*targY = 0;
}



bool Target::targetCentred(double* targX, double* targY)
{
	getTarget(targX, targY);

	double dist;
	if(fovea(*targX, *targY, &dist))
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool Target::targetCentred(double* targX, double* targY, const string colourTest)
{
	getTarget(targX, targY, colourTest);

	double dist;
	return fovea(*targX, *targY, &dist);

}

bool Target::targetCentred()
{
	return centred;
}

bool Target::fovea(double x, double y, double* dist)
{
	int centreX = 320/2;
	int centreY = 240/2;


	*dist = sqrt((x - centreX)*(x - centreX) + (y - centreY)*(y - centreY));
	if(*dist <= 16)
	{
		centred = true;
		return true;
	}
	else
	{
		centred = false;
		return false;
	}
}


bool Target::uuGetTarget(double* targX, double* targY)
{
//	if(sentFixate)
//	{
//		*targX = lastX;
//		*targY = lastY;
//		return true;
//	}

	Time::delay(1.5);
	Bottle& status = portUU.prepare();
	status.clear();
	if(requestROI)
	{
		status.addDouble(requestNewROI);
		requestROI = false;
	}
	else
		status.addDouble(whereIsROI);
	portUU.write();

	if (target = porttargetsright.read(1))	//blocking till target available?
	{
		visible = true;
		colour = "Object";
		*targX = target->get(0).asDouble();
		*targY = target->get(1).asDouble();
		if(*targX == -1 || *targY == -1)	//No target
		{
			visible = false;
			*targX = 0;
			*targY = 0;
			printf("no target\n");
			targetlogfile << "- 0 0 No target" << endl;
			return false;
		}
		else if(*targX==-2 || *targY == -2)		//UU finished training
		{
			requestROI = true;
			sentFixate = false;
			return uuGetTarget(targX, targY);
		}
		else if(*targX==-3 || *targY == -3)		//no novel target
		{
			requestROI = true;
			sentFixate = false;
			return uuGetTarget(targX, targY);
		}
		else
		{
			*targX/=2;		//convert from 640 to 320 image
			*targY/=2;		//convert from 480 to 240 image
		}
		printf("Colour: %s  X: %.3f  Y: %.3f\n",
				colour.c_str(),
				*targX, *targY);

		targetlogfile << target->get(0).asString().c_str() << " " <<
				*targX << " " << *targY;

		double dist;
		if(fovea(*targX, *targY, &dist))
		{
//			if(!sentFixate)
//			{
				Bottle& fixed = portUU.prepare();
				fixed.clear();
				fixed.addDouble(fixated);
				portUU.write();
				sentFixate = true;
//			}
			printf("Central\n");
			targetlogfile << " " << dist << " Central";
		}
		else
			targetlogfile << " " << dist;
		targetlogfile << endl;
		lastX = *targX;
		lastY = *targY;

		return true;

	}
	else
	{
		visible = false;
		*targX = 0;
		*targY = 0;
		printf("no target\n");
		targetlogfile << "- 0 0 No target" << endl;
		return false;
	}
}


bool Target::getLeft(double* targX, double* targY, string colourTest)
{
	porttargetsleft.read(0);	//Clears last target for when target is no longer visible
	Time::delay(0.15);
	bool found = false;
	if(target = porttargetsleft.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numColours = size/elements;
			for(int i=0; i<numColours; i++)
			{
				string col;
				col = target->get(i*elements).asString();
				if(col.compare(colourTest)==0)
				{
					visible = true;
					colour = target->get(i*elements).asString();
					*targX = target->get(i*elements +1).asDouble();
					*targY = target->get(i*elements +2).asDouble();
					size   = target->get(i*elements +3).asInt();
					printf("LEFT Colour: %s  X: %.3f  Y: %.3f\n",
							colour.c_str(),
							*targX, *targY);

					targetlogfile << colour << " " <<
							*targX << " " << *targY;

					double dist;
					if(fovea(*targX, *targY, &dist))
					{
						printf("Central\n");
						targetlogfile << " " << dist <<" Central";
					}
					else
						targetlogfile << " " << dist;
					targetlogfile << endl;
//					lastX = *targX;
//					lastY = *targY;
					return true;
				}
			}
		}
	}
	visible = false;
	*targX = 0;
	*targY = 0;
	printf("no target\n");
	targetlogfile << "- 0 0 No target of specified colour" << endl;
	return false;
}


void Target::getLastTargs(double* lstX, double* lstY)
{
	*lstX = lastX;
	*lstY = lastY;
}


void Target::initLog(string path)
{
	string fullpath = path + "targetlog.txt";

	targetlogfile.open(fullpath.c_str());
	targetlogfile << "Colour x y distFromCentre comment" << endl;
}

void Target::logSaccadeBreak(int saccadeCount)
{
	targetlogfile << "*******************************" << endl;
	targetlogfile << "Starting saccade: " << saccadeCount << endl;
	if(ulster)
	{
		requestROI = true;
		sentFixate = false;
	}
}

void Target::closeLog()
{
	targetlogfile << "**************END**************" << endl;
	targetlogfile.close();
}

#endif

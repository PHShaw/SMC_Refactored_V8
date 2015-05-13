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
	initTarget(params.m_ROBOT);
	visible = false;


	if(BASIC_VISION)
	{
		colourElements = 4; // [colour] [x] [y] [pixels]
		elementsPerTargetTypes["colour"] = 4;
	}
	else
	{
		colourElements = 5; // "colour" [colour] [x] [y] [pixels]
		elementsPerTargetTypes["colour"] = 5;
		elementsPerTargetTypes["shape"] = 7;  //"shape" [square|circle|triangle] [colour] [x] [y] [width] [height]
		elementsPerTargetTypes["edge"] = 12;  //"edge" [x_centre] [y_centre] [num_pts=4] [x1][y1][x2][y2]... (default 4 points)
		elementsPerTargetTypes["motion"] = 6; //"motion" [speed px/s] [x0] [y0] [x1] [y1]
	}


}


bool Target::targetVisible()
{
	return visible;
}

bool Target::initTarget(string robot)
{
	Network yarp;

	if(robot.compare("icubSim")==0)
	{
		porttargetsleft.open("/targetSim/left/read");
		porttargetsright.open("/targetSim/right/read");

//		yarp.connect("/targetSim/left/data", "/targetSim/left/read");
//		yarp.connect("/targetSim/right/data", "/targetSim/right/read");
	}
	else
	{
		porttargetsleft.open("/target/left/read");
		porttargetsright.open("/target/right/read");

//		yarp.connect("/target/left/data", "/target/left/read");
//		yarp.connect("/target/right/data", "/target/right/read");
	}

	centred = false;
	return true;
}

int Target::getNumTargets(std::string type)
{
	if(numVisibleTargets.find(type)==numVisibleTargets.end())
		return 0;
	return numVisibleTargets[type];
}

bool Target::getTarget(double* targX, double* targY)
{

//	porttargetsright.read(0);	//Clears last target for when target is no longer visible
	Time::delay(0.15);
	if (target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			numTargets=0;
			if(BASIC_VISION)
			{
				numVisibleTargets.clear();
				numTargets = size/colourElements;
				numVisibleTargets["colour"] = numTargets;
			}
			else{
				int numElements;
				numVisibleTargets.clear();
				for(int i=0; i<size;)
				{
					numTargets++;
					string type = target->get(i).asString().c_str();
					numElements = elementsPerTargetTypes[type];
					if(numVisibleTargets.find(type)==numVisibleTargets.end())
					{
						numVisibleTargets[type] = 0;
					}
					numVisibleTargets[type] ++;

					i+= numElements;
				}
			}

			int col = randGenerator(numTargets);
			printf("***********Number of targets detected: %i, Target selected: %i*************\n", numTargets, col);

			if(BASIC_VISION)
			{
				col *= colourElements;
				visible = true;
				colour = target->get(0+col).asString();
				*targX = target->get(1+col).asDouble();
				*targY = target->get(2+col).asDouble();
				size   = target->get(3+col).asInt();
				printf("Colour: %s  X: %.3f  Y: %.3f\n",
						colour.c_str(),
						*targX, *targY);

				targetlogfile << colour << " " << *targX << " " << *targY;
			}
			else
			{
				//TODO need to consider vision flags when selecting target.
				//TODO More advanced visual features should be preferred over simpler features.

				bool motion=false;
				if(getNumTargets("motion")>0 && (params.m_VISION_FLAGS & MOTION))
				{
					motion=true;
					//selection a motion target
					col = randGenerator(numVisibleTargets["motion"]);
				}


				int targetNum=0;
				int numElements=0;
				int index=0;
				while(index<size && targetNum!=col)
				{

					string type = target->get(index).asString().c_str();
					numElements = elementsPerTargetTypes[type];
					index+= numElements;
					if(motion && type=="motion")
						targetNum++;
					else if(!motion && type=="colour")
						targetNum++;
				}
				string type = target->get(index).asString().c_str();
				if(type.compare("colour")==0)
				{
//					["colour" colour x y size]
					visible = true;
					colour = target->get(1+index).asString();
					*targX = target->get(2+index).asDouble();
					*targY = target->get(3+index).asDouble();
					size   = target->get(4+index).asInt();
					printf("Colour: %s  X: %.3f  Y: %.3f\n",
							colour.c_str(),
							*targX, *targY);

					targetlogfile << colour << " " << *targX << " " << *targY;
				}
				else if(type.compare("motion")==0)
				{
//					["motion" speed x0 y0 x1 y1]
					//TODO: Make use of the starting point and speed.
					visible = true;
					colour = "motion";
					*targX = target->get(4+index).asDouble();
					*targY = target->get(5+index).asDouble();
					size = target->get(2+index).asDouble();
					printf("Motion: %s  X: %.3f  Y: %.3f\n",
							colour.c_str(),
							*targX, *targY);

					targetlogfile << colour << " " << *targX << " " << *targY;

				}
				else if(type.compare("edge")==0)
				{
//					["edge" x_centre y_centre points(4) x1 y1 x2 y2 ...]
					//centre coordinates and edge coordinates give.
					//randomly selecting one of the 4 coordinates to set as target
					size = target->get(3+index).asDouble();	//TODO: Check these indicies are correct
					int edge = randGenerator(size);
					visible = true;
					colour = "edge";
					*targX = target->get(4+index+edge*2).asDouble();
					*targY = target->get(4+index+edge*2+1).asDouble();
					printf("Edge: %s  X: %.3f  Y: %.3f\n",
							colour.c_str(),
							*targX, *targY);

					targetlogfile << colour << " " << *targX << " " << *targY;

				}
				else if(type.compare("shape")==0)
				{
//					["shape" shapeType[square|circle|triangle] colour x y w h]
					//TODO: Make better use of shape information.
					visible = true;
					colour = target->get(1+index).asString();	//shape
					*targX = target->get(3+index).asDouble();
					*targY = target->get(4+index).asDouble();
					size = target->get(5+index).asDouble();
					printf("Shape: %s  X: %.3f  Y: %.3f\n",
							colour.c_str(),
							*targX, *targY);

					targetlogfile << colour << " " << *targX << " " << *targY;

				}

			}


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

			for(map<string,int>::iterator it=numVisibleTargets.begin(); it!=numVisibleTargets.end(); it++)
			{
				cout << "[" << it->first << ":" << it->second << "] ";
			}
			cout << endl;

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
		numTargets = 0;
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

//	porttargetsright.read(0);	//Clears last target for when target is no longer visible
	Time::delay(0.15);
	bool found = false;
	if(target = porttargetsright.read(0))
	{
		int size = target->size();	//[colour, x, y]
		if(size>0)
		{
			int numTargets;
			if(BASIC_VISION)
				numTargets = size/colourElements;
			else{
				int numElements;
				for(int i=0; i<size;)
				{
					numTargets++;
					string type = target->get(i).asString().c_str();
					numElements = elementsPerTargetTypes[type];

					i+= numElements;
				}
			}

			if(BASIC_VISION)
			{
				for(int i=0; i<numTargets; i++)
				{
					string col;
					col = target->get(i*colourElements).asString();
					if(col.compare(colourTest)==0)
					{
						visible = true;
						colour = target->get(i*colourElements).asString();
						*targX = target->get(i*colourElements +1).asDouble();
						*targY = target->get(i*colourElements +2).asDouble();
						size   = target->get(i*colourElements +3).asInt();
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
			else
			{

				int numElements=0;
				int index=0;
				while(index<size)
				{
					numTargets++;
					string type = target->get(index).asString().c_str();
					numElements = elementsPerTargetTypes[type];

					index+= numElements;



					if(type.compare("colour")==0)
					{
						visible = true;
						colour = target->get(1+index).asString();
						if(colour.compare(colourTest)==0)
						{
							*targX = target->get(2+index).asDouble();
							*targY = target->get(3+index).asDouble();
							size   = target->get(4+index).asInt();
							printf("Colour: %s  X: %.3f  Y: %.3f\n",
									colour.c_str(),
									*targX, *targY);

							targetlogfile << colour << " " << *targX << " " << *targY;
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
						else
							continue;
					}
					else if(type.compare("motion")==0)
					{
						if(colourTest.compare(type)==0)
						{
							//TODO: Make use of the starting point and speed.
							visible = true;
							colour = "motion";
							*targX = target->get(4+index).asDouble();
							*targY = target->get(5+index).asDouble();
							size = target->get(2+index).asDouble();
							printf("Motion: %s  X: %.3f  Y: %.3f\n",
									colour.c_str(),
									*targX, *targY);

							targetlogfile << colour << " " << *targX << " " << *targY;
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
						else continue;
					}
					else if(type.compare("edge")==0)
					{
						if(colourTest.compare(type)==0)
						{
							//centre coordinates and edge coordinates give.
							//randomly selecting one of the 4 coordinates to set as target
							size = target->get(3+index).asDouble();	//TODO: Check these indices are correct
							int edge = randGenerator(size) *2;
							visible = true;
							colour = "edge";
							*targX = target->get(4+index+edge).asDouble();
							*targY = target->get(4+index+edge+1).asDouble();
							printf("Edge: %s  X: %.3f  Y: %.3f\n",
									colour.c_str(),
									*targX, *targY);

							targetlogfile << colour << " " << *targX << " " << *targY;
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
						else continue;
					}
					else if(type.compare("shape")==0)
					{
						colour = target->get(1+index).asString();
						if(colourTest.compare(colour)==0)
						{
							//TODO: Make better use of shape information.
							visible = true;

							*targX = target->get(3+index).asDouble();
							*targY = target->get(4+index).asDouble();
							size = target->get(5+index).asDouble();
							printf("Shape: %s  X: %.3f  Y: %.3f\n",
									colour.c_str(),
									*targX, *targY);

							targetlogfile << colour << " " << *targX << " " << *targY;
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
						else
							continue;
					}
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
			int numColours = size/colourElements;
			for(int i=0; i<numColours; i++)
			{
				visible = true;
				string col = target->get(i*colourElements).asString().c_str();
				*targX = target->get(i*colourElements +1).asDouble();
				*targY = target->get(i*colourElements +2).asDouble();
				size   = target->get(i*colourElements +3).asInt();

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
			int numColours = size/colourElements;
			double distances[numColours];

			for(int i=0; i<numColours; i++)
			{
				visible = true;
				string col = target->get(i*colourElements).asString().c_str();
				targX = target->get(i*colourElements +1).asDouble();
				targY = target->get(i*colourElements +2).asDouble();
				size   = target->get(i*colourElements +3).asInt();

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
				coloursDist.push_back(target->get(id*colourElements).asString().c_str());
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
			int numColours = size/colourElements;
			for(int i=0; i<numColours; i++)
			{
				visible = true;
				string col = target->get(i*colourElements).asString().c_str();
				*targX = target->get(i*colourElements +1).asDouble();
				*targY = target->get(i*colourElements +2).asDouble();
				size   = target->get(i*colourElements +3).asInt();

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
	int centreX = RETINA_WIDTH/2;
	int centreY = RETINA_HEIGHT/2;


	*dist = sqrt((x - centreX)*(x - centreX) + (y - centreY)*(y - centreY));
	if(*dist <= 0.05*RETINA_WIDTH)
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
			int numColours = size/colourElements;
			for(int i=0; i<numColours; i++)
			{
				string col;
				col = target->get(i*colourElements).asString();
				if(col.compare(colourTest)==0)
				{
					visible = true;
					colour = target->get(i*colourElements).asString();
					*targX = target->get(i*colourElements +1).asDouble();
					*targY = target->get(i*colourElements +2).asDouble();
					size   = target->get(i*colourElements +3).asInt();
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
}

void Target::closeLog()
{
	targetlogfile << "**************END**************" << endl;
	targetlogfile.close();
}

#endif

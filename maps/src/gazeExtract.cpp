/*
 * GazeExtract.cpp
 *
 *  Created on: 5 Jul 2011
 *      Author: icub
 */



#include "Gaze_IO.h"


#include <fstream>
#include <iostream>
#include <vector>


int main(int argc, char* argv[])
{

	string path;
	if(argc >=2)
	{
		path = string(argv[1]);
		cout << "loading files from path: " << path << endl;
	}
	else
	{
		cout << "Enter the path to load eye and head links from:" <<endl;
		cin >> path;
	}


	GazeMap* gm = new GazeMap();


	Gaze_IO GIjoe;

	gm = GIjoe.loadMappingFromXML(path + "GM_testXV10");

	//*********GAZE FIELDS************//
	ofstream gazeFields;
	string fullname = path + "gazeFieldsXV10.txt";
	gazeFields.open(fullname.c_str());

	vector<GazeField*> fields = gm->getGazeFields();
	size_t numFields = fields.size();

	for(size_t i = 0; i<numFields; i++)
	{
		GazeField* field = fields[i];
		double gazeX, gazeY;
		field->getGazeDirection(&gazeX, &gazeY);
		gazeFields << gazeX << " " << gazeY << endl;
	}
	gazeFields.close();


	//*********REACH FIELDS***********//
	ofstream reachFile;
	fullname = path + "reachFieldsXV10.txt";
	reachFile.open(fullname.c_str());
	vector<ReachField*> reachFields = gm->getReachFields();
	for(size_t i=0; i<reachFields.size(); i++)
	{
		double x,y;
		ReachField* temp = reachFields.at(i);
		temp->getReachDirection(&x, &y);
		reachFile << x << " " << y << endl;
	}
	reachFile.close();


	//*********LINKED FIELDS***********//
	ofstream linkedFields;
	fullname = path + "gazeReachFieldsXV10.txt";
	linkedFields.open(fullname.c_str());
	vector<GazeReachLink*> links = gm->getLinks();
	for(size_t i=0; i<links.size(); i++)
	{
		GazeReachLink* l = links.at(i);

		linkedFields << l->getGazeX() 	   << " " << l->getGazeY() 	  << " " <<
						l->getArmFlexion() << " " << l->getArmAngle() << " " << l->gazeDepth;// << endl;

		cout << "Gaze: " << *(l->gaze) << ", Reach: " << *(l->reach) << endl;

		bool rightArm;
		ReachConfig* rc = l->reach->getPreferredReach(&rightArm);
		linkedFields << " " << rc->shoulderPitch << " " << rc->shoulderRoll << " " << rc->shoulderYaw << " " << rc->elbow << " " << rc->wristProSup << " " << rc->wristPitch << endl;
	}
	linkedFields.close();

	delete gm;
	return (0);

}

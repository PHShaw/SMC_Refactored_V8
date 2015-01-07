/*
 * gazeTester.cpp
 *
 *  Created on: 5 Apr 2011
 *      Author: icub
 */

#include <iostream>

#include "GazeMap.h"

#include "Gaze_IO.h"

using namespace std;
double ARM_BOARD_REST[] =	{-38, 64,  50, 15,  20,-30,  0, 60,  6, 55, 30,  0,  0,  0,  0,  0};

int randGenerator(int range)
{

	return rand() % range;
}


int main()
{
	srand( time(NULL));
/*
	//Test head Struct first
	double in;
	double* headArray = new double[6];
	cout << "Enter 6 doubles for a head configuration" << endl;
	for (int i=0; i<6; i++)
	{
		cin >> in;
		headArray[i] = in;
	}

	headConfig* hc = new headConfig(headArray);
	cout << *hc << endl;

	double* config = new double[6];
	hc->toArray(config);
	for(int i=0; i<6; i++)
	{
		cout << config[i] << endl;
	}


	cout << "Adding gaze to gazeField" << endl;
	gazeField* gf = new gazeField();
	gf->addGazeConfiguration(config);

	cout << "Enter 6 doubles for a head configuration" << endl;
	for (int i=0; i<6; i++)
	{
		cin >> in;
		headArray[i] = in;
	}
	gf->addGazeConfiguration(headArray);

	cout << *gf << endl;

	hc = gf->getPreferredGaze();
	cout << "Preferred: " << *hc << endl;


	vector<gazeField*> gazes;
	gazes.push_back(gf);


	gazeField* gf2 = new gazeField();
	for(int i=0; i<3; i++)
	{
		cout << "Enter 6 doubles for a head configuration" << endl;
		headArray = new double[6];
		for (int i=0; i<6; i++)
		{
			cin >> in;
			headArray[i] = in;
		}
		gf2->addGazeConfiguration(headArray);
	}

	gazes.push_back(gf2);


	gazeField* gf3 = new gazeField();
	for(int i=0; i<5; i++)
	{
		cout << "Enter 6 doubles for a head configuration" << endl;
		headArray = new double[6];
		for (int i=0; i<6; i++)
		{
			cin >> in;
			headArray[i] = in;
		}
		gf3->addGazeConfiguration(headArray);
	}
	gazes.push_back(gf3);


	for(int i=0; i<gazes.size(); i++)
	{
		gazeField* g = gazes.at(i);
		cout << *g << endl;
		headConfig *h = g->getPreferredGaze();
		cout << *h << endl;
	}

*/


	cout << "Now attempting to test the gaze map" << endl;

	Gaze_IO gmIO;


	GazeMap* gm = gmIO.loadMappingFromXML("GM_testXV9");	//gm_io adds type and .xml to filename
//	GazeMap* gm = new GazeMap();
	double in;
	double* headConfig = new double[6];
	double* reachConfig = new double[16];

	for(int j=0; j<100; j++)
	{
//		cout << "Enter 6 doubles for a head configuration" << endl;
		headConfig = new double[6];
		reachConfig = new double[16];
		for (int i=0; i<6; i++)
		{
//			cin >> in;
			in = randGenerator(15);
			headConfig[i] = in;

		}
//		cout << "Enter 16 doubles for a reach configuration" << endl;
		for (int i=0; i<16; i++)
		{
//			cin >> in;
			in = randGenerator(15);
			reachConfig[i] = in;
		}

//		cout << "Adding " << j << endl;
		gm->addGazeReach(headConfig, reachConfig, true, 300);
	}

	cout << "looking for reach config" << endl;
	reachConfig = new double[16];
	for (int i=0; i<16; i++)
	{
//		cin >> in;
		in = randGenerator(15);
		reachConfig[i] = in;
	}

	ReachField* rf = gm->getReachField(reachConfig,true);
	cout << *rf << endl;

	GazeField* gf = new GazeField();
	bool result = gm->getEyeCoordination(rf, &gf);



	cout << result << endl;
//	cout << "Correlation: " << corr << endl;
	cout << *gf << endl;


	reachConfig = new double[16];
	for (int i=0; i<16; i++)
	{
//		cin >> in;
		in = randGenerator(15);
		reachConfig[i] = in;
	}

	double corr;// = 200;
	GazeField* field = new GazeField();
	gm->getNearestGazeFieldForArmReach(reachConfig,true,&field, &corr);
	cout << "Correlation: " << corr << endl;
	cout << *field << endl;
//
//	cout << "Enter 6 doubles for a head configuration" << endl;
//	for (int i=0; i<6; i++)
//	{
//		cin >> in;
//		headConfig[i] = in;
//
//	}

//	gm->addGazeField(headConfig);



	gmIO.saveMappingToXML(gm, "GM_testXV9");	//gm_io adds type and .xml to filename

	delete gm;


//	cout << "Testing reach configurations" << endl;
//
//
//
//	double in;
//	double* reachArray = new double[16];
//	cout << "Enter 16 doubles for a reach configuration" << endl;
//	for (int i=0; i<16; i++)
//	{
//		cin >> in;
//		reachArray[i] = in;
//	}
//	reachConfig* rc = new reachConfig(reachArray);
//	cout << *rc << endl;
//
//	double* config = new double[16];
//	rc->toArray(config);
////	for(int i=0; i<16; i++)
////	{
////		cout << config[i] << endl;
////	}
//
//
//	cout << "Adding reach to gazeField" << endl;
//	gazeField* gf = new gazeField();
//	gf->addReachConfiguration(config, true);
//
//	cout << "Enter 16 doubles for a reach configuration" << endl;
//	for (int i=0; i<16; i++)
//	{
//		cin >> in;
//		reachArray[i] = in;
//	}
//	gf->addGazeConfiguration(reachArray);
//
//	cout << *gf << endl;
//
//	bool rightarm = true;
//	rc = gf->getPreferredReach(&rightarm);
//	cout << "Preferred: " << *rc << endl;
}

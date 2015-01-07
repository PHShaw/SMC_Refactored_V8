/*
 * learningMenu.cpp
 *
 *  Created on: 17 Dec 2012
 *      Author: Patricia Shaw
 *      			Aberystwyth University
 *
 */

#include <signal.h>		//used in the save and quit functions

#include "armReaching.h"
#include "EyeHeadController.h"
#include "handEyeCoordination.h"
#include "ShortTermMemoryV2.h"

#include "tracker.h"

#include "graspController.h"
#include "TorsoReach.h"

#include "testFunctions.cpp"

/**
 * Startup routine:
 * 0. create target
 * 1. create hand eye coordination
 * a. get path and filenames if needing to load up maps.
 * 2. load gazeReachMap, returning pointer to it
 * 3. create eye head controller
 * 4. create arm reaching
 * 5. init hand eye coordination with eye head controller and arm reaching
 */

armReaching* armReach;
EyeHeadSaccading* ehCont;
graspController* grippy;
handEyeCoordination* heCoor;
Target* target;
torsoSaccading* tor;

/**
 * In experiments for sequential vs synchronous learning, these variable should be changed!
 * Moving to params_config.h
 */
//bool synchronous = true;
//bool nearestNeighbour = true;
//bool safeMode = true;


/**
 * End variable
 */



//1. Run $ICUB_ROOT/app/faceExpressions/scripts/app.xml.template
//2. Run ./emotionController
enum emote {neu,hap1,hap2,sad,sur,ang,evi,shy,cun};
yarp::os::BufferedPort<yarp::os::Bottle> portOut;
void write(emote e)
{
	yarp::os::Bottle& out = portOut.prepare();
	out.clear();
	out.addInt(e);
	cout << "Sending int " << e << endl;
	portOut.write(true);
	yarp::os::Time::delay(0.001);
}


void saveAndQuit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
	heCoor->closeLogs();

	ehCont->saveMaps();
	tor->saveMapping();
	heCoor->saveGazeReachMap();


	std::exit(param);
}


//This function is used as above, but doesn't save the maps,
//i.e. when learning not enabled
void quit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
	delete ehCont;
	delete tor;

	std::exit(param);
}



int main(int argc, char* argv[])
{
	srand( time(NULL));

	yarp::os::Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	yarp::os::Value* val;
	if(options.check("robot",val))
	{
		params.m_ROBOT = val->asString().c_str();
		cout << "Selected robot: " << params.m_ROBOT << endl;
	}
	else
	{
		cout << "A robot can be specified from the command line e.g. --robot [icub|icubSim]" << endl;
		params.m_ROBOT = "icubSim";
	}

	target = new Target(params.m_ROBOT);

	params.m_LOAD = false;
//	string path, filename;
//	bool load = false;


	if(options.check("path",val))
	{
		path = val->asString().c_str();
		params.m_LOAD = true;
		cout << "Loading files from path: " << path << endl;

		if(options.check("name",val))
		{
			params.m_FILENAME = val->asString().c_str();
			cout << "Loading file set: " << filename << endl;
		}
		else
		{
			cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> params.m_FILENAME;
		}
	}
	else
	{
		cout << "Would you like to load an existing mapping? y/n" << endl;
		char loading;
		cin >> loading;

		if (loading == 'y')
		{

			cout << "Enter the path to the directory containing the files: e.g. ../data/ " <<endl;
			cin >> params.m_PATH;
			cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> params.m_FILENAME;
			params.m_LOAD = true;
		}
		else
		{
			cout << "Enter the path to the directory to which log should be placed: e.g. ../data/ " <<endl;
			cin >> params.m_PATH;
			cout << "Enter a generic name to save the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> params.m_FILENAME;
	//		path = "../data/";
	//		filename = "testXV10";
		}
	}

	target->initLog(params.m_PATH);
	cout << "Would you like to enable learning? y/n" << endl;
	char learning;
	cin >>learning;
	bool learn = false;
	if(learning == 'y')
	{
		learn = true;
	}
//	if(load)
//	{
		heCoor = new handEyeCoordination();
		GazeMap* gm = heCoor->getGazeMap();
		ehCont = new EyeHeadSaccading(gm, target);
		tor = new torsoSaccading(ehCont, target);
		armReach = new armReaching(gm);
//	}
//	else
//	{
//		heCoor = new handEyeCoordination(load);
//		GazeMap* gm = heCoor->getGazeMap();
//		ehCont = new EyeHeadController(gm, target, synchronous, nearestNeighbour, load);
//		armReach = new armReaching(gm, learn, safeMode);
//	}

	if(learn)
		signal(SIGINT, saveAndQuit);
	else
		signal(SIGINT, quit);





	heCoor->init(ehCont, armReach, target);
//	grippy = new graspController(robot, armReach->getArmController());



	yarp::os::Network yarp;

	portOut.open("/emote/ask");
	yarp.connect("/emote/ask", "/emote/emotion");


//now ready to start learning
	int choice = 0;
	const int exit = 11;
	while(choice!=exit){

		cout << "\n\nSelect an option from the menu: [1-8]\n" <<
				"1. Learn eye mapping\n" <<
				"2. Learn eye-head mapping\n" <<
				"21. Learn mega head links\n" <<
				"3. Learn hand eye coordination with predefined arm configurations\n" <<
				"4. Learn custom hand eye coordination\n" <<
				"5. Test all hand-eye coordination\n" <<
				"6. Test specific hand-eye coordination\n" <<
				"65. Check reach for gaze\n" <<
				"7. Refine hand-eye coordination\n" <<
				"8. Learn torso control - requires matlab\n" <<
				"9. Test world target coords\n" <<
				"10. Test torso control - requires matlab\n" <<
				exit << ". Exit" << endl;

		cout << "20. Record microdynamics of VOR" << endl;
		cout << "128. Learn eye+head+torso control for filming" << endl;
		cout << "666. Test gaze world coords" << endl;
		cin >> choice;

		switch(choice)
		{

			case 1:
			{
//***************************************************************************************
//	EYE ONLY LEARNING:
//
				if(!SYNCHRONOUS)
				{
					int numEyeSaccades = ehCont->learnEyeSaccades();
					cout << "Learnt eye saccades after " << numEyeSaccades << " attempts" << endl;
					ehCont->saveMaps();
				}
//***************************************************************************************
				break;
			}

			case 2:
			{
//***************************************************************************************
//	EYE + HEAD SACCADE LEARNING:
//
				int numHeadSaccades = ehCont->learnEyeHeadSaccades();
				cout << "Learnt eye-head saccades after " << numHeadSaccades << " attempts, " <<
						"with a maximum set at 100" << endl;
				ehCont->saveMaps();
//***************************************************************************************
				break;
			}

			case 21:
				//ehCont->learnOuterHeadLinks();
				ehCont->learn_iStyleHeadLinks();
				ehCont->saveMaps();
				break;


			case 3:
			{
//***************************************************************************************
//	ARM REACH LEARNING:
//
				cout << "Each arm should be learnt independently.  " <<
						"Which arm do you want to learn now? [r/l]" << endl;
				char c;
				bool rightArm;
				cin >> c;
				if(c == 'r')
					rightArm = true;	//learn right arm
				else
					rightArm = false;	//learn left arm

				tracker* t = new tracker();
				t->initTrack(ehCont->getMotorDriver(), target, ehCont);
				boost::thread trackThrd(boost::bind(&tracker::track,t));


				int coord = heCoor->learnCoordination(rightArm);
				cout << "Learnt some hand eye coordination from " << coord << " attempts" << endl;

				trackThrd.interrupt();
				heCoor->saveGazeReachMap();
				armReach->toRest();
//***************************************************************************************
				break;
			}


//***************************************************************************************
//Testing and customisation of hand eye coordination
			case 4:
			{
				cout << "Each arm should be learnt independently.  " <<
						"Which arm do you want to learn now? [r/l]" << endl;
				char c;
				bool rightArm;
				cin >> c;
				if(c == 'r')
					rightArm = true;	//learn right arm
				else
					rightArm = false;	//learn left arm

				addReachPositions(rightArm, heCoor);	//learn a set of manual positions for the left arm
				heCoor->saveGazeReachMap();
				break;
			}

			case 5:
				testHandEyeCoordination(heCoor);
				break;

			case 6:
				testSpecificHandEyeCoordination(heCoor);
				break;

			case 65:
			{

				GazeField* gf = ehCont->getGazeField();
				cout << "Gaze field: " << *gf << endl;
				ReachField* rf = new ReachField();
				double dist;
				heCoor->getNearestReachForGaze(gf, &rf, &dist);
				cout << "Reach field: " << *rf << endl;
				cout << "Dist: " << dist << endl;
				bool rightArm;
				ReachConfig* rc = rf->getPreferredReach(&rightArm);
				armReach->goToReachConfig(rc);

				yarp::os::Time::delay(1.0);
				break;
			}


			case 7:
				refineCoordination(heCoor);
				heCoor->saveGazeReachMap();
				break;
//***************************************************************************************

			case 8:
			{
//***************************************************************************************
//Torso LWPR training using matlab
				tor->initMatlabPorts();
				tor->LWPR_TorsoLearnerFullModel();
				tor->closeMatlabPorts();
				break;
//***************************************************************************************
			}

			case 9:
				testTargetCoords(ehCont);
				break;

			case 10:
			{
				tor->initMatlabPorts();
				string colour = "red";
				double depth;
				ehCont->fixate(colour);
				ehCont->verge(colour, &depth);

				double gazeX, gazeY, vergence;
				ehCont->getGazeDirection(&gazeX, &gazeY);
				ehCont->getVergence(&vergence);
				cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;

//				cout << "Which hand? [l/r]" << endl;
//				char c;
//				cin >> c;
//				bool rightHand;
//				if(c=='r')
//					rightHand=true;
//				else
//					rightHand=false;

				cout << "Enter desired values for gazeX gazeY and vergence:" << endl;
				double desiredGazeX, desiredGazeY, desiredVergence;
				cin >> desiredGazeX >> desiredGazeY >> desiredVergence;


				cout << "Initialising tracker" << endl;
				tracker* t = new tracker(colour, ehCont);

				cout << "Starting tracker thread" << endl;
				boost::thread trackThrd(boost::bind(&tracker::track,t));
				//Use torso to bring target into reach

				cout << "Requesting torso reach" << endl;
				tor->LWPR_TorsoReach(desiredGazeX, desiredGazeY, desiredVergence);

				cout << "Waiting for torso-head move to finish" << endl;
				yarp::os::Time::delay(1.5);

				cout << "Interrupting tracker thread" << endl;
				trackThrd.interrupt();
				yarp::os::Time::delay(1.5);

				tor->closeMatlabPorts();

				ehCont->fixate(colour);
				ehCont->verge(colour, &depth);

//				double gazeX, gazeY, vergence;
				ehCont->getGazeDirection(&gazeX, &gazeY);
				ehCont->getVergence(&vergence);
				cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;
				break;
			}


			case exit:
				cout << "exiting" << endl;
				break;


			case 128:
			{
				tor->initMatlabPorts();
				ehCont->getHeadController()->move(0, -15, true);

				cout << "+++++ Starting eye learning +++++" << endl;
				int numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "Learnt eye saccades after " << numEyeSaccades << " attempts" << endl;
				ehCont->saveMaps();

				cout << "+++++ Starting head learning +++++" << endl;
				int numHeadSaccades = ehCont->learnEyeHeadSaccades();
				cout << "Learnt eye-head saccades after " << numHeadSaccades << " attempts, " <<
						"with a maximum set at 100" << endl;
				ehCont->saveMaps();

				cout << "+++++ Starting torso learning +++++" << endl;
				tor->LWPR_TorsoLearnerFullModel();

				tor->closeMatlabPorts();
				break;
			}

			case 666:
			{
				vector<GazeField*> fields = gm->getGazeFields();
				for(int i=0; i<fields.size(); i++)
				{
					GazeField* temp = fields.at(i);
					cout << temp->wx << ", " << temp->wy << ", " << temp->wz << ", " << temp->vergence << endl;
				}
				break;
			}

			case 20:	//Record microdynamics
			{
				ehCont->getHeadController()->move(-20,0,true);
				ehCont->getEyeController()->move(-20, 0, true);

				vor* v = ehCont->getVor();
				v->resetOverflow();
				boost::thread vorThrd(boost::bind(&vor::track,v));
				yarp::os::Time::delay(0.02);
				ehCont->getHeadController()->move(20, 0, true);

				yarp::os::Time::delay(0.5);
				vorThrd.interrupt();
				yarp::os::Time::delay(0.3);
			}


			default:
				cout << "Unrecognised option, please try again" << endl;
				break;

//***************************************************************************************


		}
	}




	if(learn)
		saveAndQuit(0);
	else
		quit(0);
}

/*
 * learningMenu.cpp
 *
 *  Created on: 17 Dec 2012
 *      Author: Patricia Shaw
 *      			Aberystwyth University
 *
 */

#define MONITOR


#include <signal.h>		//used in the save and quit functions

#include "EyeHeadController.h"
#include "handEyeCoordination.h"
#include "ShortTermMemoryV2.h"

#include "tracker.h"

#include "graspController.h"
#include "torsoSaccading.h"
#include "VamTarget.h"
#include "monitor.h"


/**
 * Startup routine:
 * 0. create target
 * 1. create hand eye coordination
 * a. get path and filenames if needing to load up maps.
 * 2. load gazeReachMap, returning pointer to it
 * 3. create eye head controller
 * 4. create arm reaching
 */

EyeHeadSaccading* ehCont;
//graspController* grippy;
handEyeCoordination* heCoor;
armReaching* armReach;
Target* target;
VamTarget* vam;
torsoSaccading* tor;
GazeMap* gm;

string path;
string filename;

yarp::os::BufferedPort<yarp::os::Bottle> portOut;
yarp::os::BufferedPort<yarp::os::Bottle> portIn;
yarp::os::Bottle* reachResponse;

/**
 * Attempts to get the world coordinates of an object w.r.t. the base of the torso.
 * Returns success of saccade.  Coordinates only set if successful, and are given in mm.
 */
bool getGazeCoordinates(double *x, double *y, double *z);
bool getTargetCoordinates(string colour, double *x, double *y, double *z, bool check=true);
bool getTargetCoordinates(double *x, double *y, double *z);
bool reachToPoint(double wx, double wy, double wz, double* dist);
void command(string cmd);
void command(string cmd, bool arm);
void torsoMove(double wx, double wy, double wz);
bool torsoTest(string suffix, int torStartX, int torStartY,
		double desiredGazeX, double desiredGazeY, double desiredVergence);

string targetColour = "red";
string const handColour = "yellow";


//1. Run $ICUB_ROOT/app/faceExpressions/scripts/app.xml.template
//2. Run ./emotionController
enum emote {neu,hap1,hap2,sad,sur,ang,evi,shy,cun};
yarp::os::BufferedPort<yarp::os::Bottle> emotePort;
void write(emote e)
{
	yarp::os::Bottle& out = emotePort.prepare();
	out.clear();
	out.addInt(e);
	cout << "Sending int " << e << endl;
	emotePort.write(true);
	yarp::os::Time::delay(0.001);
}


void loadGazeReachMap()
{
	Gaze_IO gm_io;
	delete gm;
	gm = gm_io.loadMappingFromXML(path + "GM_" + filename);
	cout << "There are " << gm->getNumGazeFields() << " fields in the gaze map" << endl;

}
void saveGazeMap()
{
	Gaze_IO gmIO;
	try{
		cout << "There are " << gm->getNumGazeFields() << " gaze fields to save" << endl;
		gmIO.saveMappingToXML(gm, path+"GM_" + filename);
		cout << "Successfully saved: ";
		cout << gm->getNumGazeFields() << " Gaze fields, ";
//		cout << gm->getNumReachFields() << " Reach fields and ";
//		cout << gm->getNumLinks() << " links " << endl;
	}
	catch(IMapException ime)
	{
		cout << "An error occurred whilst attempting to save the gaze map"<<endl;
	}
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
	saveGazeMap();

	tor->closeMatlabPorts();

	delete ehCont;
	delete tor;
	delete vam;

	std::exit(param);
}


//This function is used as above, but doesn't save the maps,
//i.e. when learning not enabled
void quit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
	tor->closeMatlabPorts();

	delete ehCont;
	delete tor;
	delete vam;


	std::exit(param);
}


int main(int argc, char* argv[])
{
	srand( time(NULL));
	yarp::os::Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	yarp::os::Value* val;
	std::string robot;
	if(options.check("robot",val))
	{
		robot = val->asString().c_str();
		cout << "Selected robot: " << robot << endl;
	}
	else
	{
		cout << "A robot can be specified from the command line e.g. --robot [icub|icubSim]+F" << endl;
		robot = "icub";
	}

	target = new Target();
	bool load = true;
//	string path, filename;

	if(options.check("path",val))
	{
		path = val->asString().c_str();
		load = true;
		cout << "Loading files from path: " << path << endl;

		if(options.check("name",val))
		{
			filename = val->asString().c_str();
			cout << "Loading file set: " << filename << endl;
		}
		else
		{
			cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> filename;
		}
	}
	else
	{

		cout << "Enter the path to the directory containing the files: e.g. ../data/ " <<endl;
		cin >> path;
		cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
		cin >> filename;
	}

	target->initLog(path);
	vam = new VamTarget();
	bool learn = true;

	heCoor = new handEyeCoordination(load, path, filename);
//	GazeMap* gm = new GazeMap();
	gm = heCoor->getGazeMap();
	loadGazeReachMap();
	ehCont = new EyeHeadSaccading(robot, gm, target, synchronous, nearestNeighbour, load, path, filename, learn);
	tor = new torsoSaccading(robot, ehCont, target, learn, nearestNeighbour, path, load, filename);
	armReach = new armReaching(robot, gm, learn, safeMode);
	armController* ac = new armController(true, robot);
	graspController* grippy;// = new graspController(robot, ac);

	if(learn)
		signal(SIGINT, saveAndQuit);
	else
		signal(SIGINT, quit);



	heCoor->init(ehCont, armReach, target, learn);
//	grippy = new graspController(armReach->getArmController());



	yarp::os::Network yarp;

	portOut.open("/smc/reaching/out");
	portIn.open("/smc/reaching/in");
	yarp.connect("/smc/reaching/out", "/aber/reach/control:i");
	yarp.connect("/aber/reach/status:o", "/smc/reaching/in");

	printf("Opening emote port...\n");
	emotePort.open("/emote/ask");
	yarp.connect("/emote/ask", "/emote/emotion");

	tor->initMatlabPorts();

#ifdef MONITOR
	monitor* m = new monitor();
	m->initMonitor(ehCont->getMotorDriver(), tor->getTorsoController()->getMotorDriver(),
			armReach->getArmController()->getLeftMotorDriver(),
			armReach->getArmController()->getRightMotorDriver());
	boost::thread monitorThrd(boost::bind(&monitor::record,m));
#endif


//now ready to start learning

	double targX, targY, gazeX, gazeY, depth;
	double worldTargX, worldTargY, worldTargZ;
	double worldHandX, worldHandY, worldHandZ;
	bool gotTarg=false, gotHand=false;
	bool rightHand=true;

	int choice = 0;
	const int exit = 14;
	while(choice!=exit){

		cout << "Select an option from the menu: [1-8]\n" <<
				"0. Stop hand\n" <<
				"1. Get gaze direction of target\n" <<
				"2. Get gaze direction of hand\n" <<
				"3. Reconnect with reaching\n" <<
				"4. Move hand to target\n" <<
				"45. Move hand to specific location\n" <<
				"5. Send current hand coordinates\n" <<
				"6. Get current status\n" <<

				"7. Try a grasp\n" <<
				"8. Release grasp\n" <<

				"9. Restart hand\n" <<
				"10. Send arms to home\n" <<
				"11. Use torso to bring target into reach\n" <<
				"12. TEST: Get gaze point in world space\n" <<
				"13. TEST: Centre torso on target\n" <<
				exit << ". Exit" << endl;

		cout << "66. Train gaze map to reaches" << endl;
		cout << "99. Torso-reach test for video" << endl;
		cout << "77. Testing vam and fancy video bits" << endl;
		cout << "110. Close matlab ports" << endl;
		cout << "112. Open matlab ports" << endl;
		cin >> choice;

		switch(choice)
		{
			case 0: //Stop the arm moving
			{
				//Stop the arm moving
				cout << "Sending stop command" << endl;
				command("stop");
				break;
			}

			case 1: //get gaze direction of target
			{
				cout << "Red/green[r/g]" <<endl;
				char c;
				cin >> c;
				if(c == 'g')
					targetColour = "green";
				else
					targetColour = "red";

				//Get gaze direction of target (red)
				gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
				gotHand=false;
				break;
			}

			case 2: //Get gaze direction of hand (yellow)
			{
				//Get gaze direction of hand (yellow)
				command("stop");
				yarp::os::Time::delay(0.1);
				gotHand = getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
				gotHand=false;
				command("restart");
				break;
			}
			case 3: //Reconnect with reaching
				yarp.connect("/smc/reaching/out", "/aber/reach/control:i");
				yarp.connect("/aber/reach/status:o", "/smc/reaching/in");
				break;

			case 4: //Move hand to target
			{
				//TODO: Now have full control over hand trajectory, so need to set way points for pre-reach positions.
				//Send current hand coordinates
				//Move hand to target
				cout << "Which hand? [l/r]" << endl;
				char c;
				cin >> c;
				if(c=='r')
					rightHand=true;
				else
					rightHand=false;

				if(!gotTarg)
				{
					gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
				}

				cout << "Target coordinates: " << worldTargX << ", " << worldTargY << ", " << worldTargZ << endl;

				cout << "Enter hand target offset values e.g. -30 40 100" << endl;
				double offsetx, offsety, offsetz;
				cin >> offsetx >> offsety >> offsetz;

				offsetx += worldTargX;
				offsety += worldTargY;
				offsetz += worldTargZ;


				cout << "requesting arm moves to coordinates" << endl;
				//calculating coordinates in mm
				//Send message to Kevin with LeftArm x y z (in m)
				yarp::os::Bottle& b = portOut.prepare();
				b.clear();
				b.addString("target");
				if(rightHand)
					b.addString("right_arm");
				else
					b.addString("left_arm");
				b.addDouble(offsetx/1000);
				b.addDouble(offsety/1000);
				b.addDouble(offsetz/1000);
				//can add "horizontal" or "vertical" at end of this message, but defaults to "horizontal"
				cout << "horizontal or vertical [h/v]" << endl;
				cin >> c;
				if(c=='v')
					b.addString("vertical");
				else
					b.addString("horizontal");
				portOut.write();

				//note that hand will move, so hand coords will no longer be valid, unless tracking and updating
				gotHand = false;

//***************************************************************************************
//	ARM REACH LEARNING:
//

//				tracker* t = new tracker();
//				t->initTrack(ehCont->getMotorDriver(), target, ehCont);
//				boost::thread trackThrd(boost::bind(&tracker::track,t));


//				int coord = heCoor->learnCoordination(rightArm);
//				cout << "Learnt some hand eye coordination from " << coord << " attempts" << endl;

//				trackThrd.interrupt();
//				heCoor->saveGazeReachMap();
//				armReach->toRest();
//***************************************************************************************
				break;
			}

			case 45: //Move hand to specific location
			{
				cout << "Which hand? [l/r]" << endl;
				char c;
				cin >> c;
				if(c=='r')
					rightHand=true;
				else
					rightHand=false;



				cout << "Enter hand target values e.g. -30 40 100" << endl;
				double x, y, z;
				cin >> x >> y >> z;


				cout << "requesting arm moves to coordinates" << endl;
				//calculating coordinates in mm
				//Send message to Kevin with LeftArm x y z (in m)
				yarp::os::Bottle& b = portOut.prepare();
				b.clear();
				b.addString("target");
				if(rightHand)
					b.addString("right_arm");
				else
					b.addString("left_arm");
				b.addDouble(x/1000);
				b.addDouble(y/1000);
				b.addDouble(z/1000);
				//can add "horizontal" or "vertical" at end of this message, but defaults to "horizontal"
				cout << "horizontal or vertical [h/v]" << endl;
				cin >> c;
				if(c=='v')
					b.addString("vertical");
				else
					b.addString("horizontal");
				portOut.write();

				//note that hand will move, so hand coords will no longer be valid, unless tracking and updating
				gotHand = false;

				break;
			}


			case 5: //Send current visual hand coordinates
			{
				command("stop");
				yarp::os::Time::delay(0.1);
				if(!gotHand)
				{
					gotHand = getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
				}


				yarp::os::Bottle& b = portOut.prepare();
				b.clear();
				b.addString("position");
				if(rightHand)
					b.addString("right_arm");
				else
					b.addString("left_arm");
				b.addDouble(worldHandX/1000);
				b.addDouble(worldHandY/1000);
				b.addDouble(worldHandZ/1000);
				portOut.write();
				gotHand = false;
				yarp::os::Time::delay(0.3);
				command("restart");
				break;
			}

			case 6: //Get current status
			{
				//Get current status
				//waiting, home, reaching, stopped, complete &dist
				reachResponse = portIn.read(true);
//				int size = reachResponse->size();
//				int pending = portIn.getPendingReads();
//				cout << "Received response of size: " << size << endl;
//				cout << "Number of pending reads: " << pending << endl;

				string message = reachResponse->get(0).asString().c_str();
				cout << "Currently reading: " << message << endl;
				break;
			}

			case 7: // try grasping
			{	// try grasping
				cout << "Which hand? [l/r]" << endl;
				char c;
				cin >> c;
				if(c=='r')
					rightHand=true;
				else
					rightHand=false;

				grippy->grasp(rightHand);
				bool success = grippy->grasp(rightHand);
				if(success)
					cout << "I think I've got it" << endl;
				else
					cout << "Hmmm, don't think that worked" << endl;

				break;
			}
			case 8: // release
			{	// release
				cout << "Which hand? [l/r]" << endl;
				char c;
				cin >> c;
				if(c=='r')
					rightHand=true;
				else
					rightHand=false;

				grippy->release(rightHand);
				break;
			}


			case 9: //restart the arm moving
			{
				//restart the arm moving
				cout << "Sending restart command" << endl;
				command("restart");
				break;
			}

			case 10: //Send the arms to the home position
			{
				//Send the arms to the home position
				cout << "Sending home command" << endl;
				command("home");
				break;
			}

			case 11: //Use the torso to bring the target into reach
			{
				double gazeX, gazeY, vergence;
				ehCont->getGazeDirection(&gazeX, &gazeY);
				ehCont->getVergence(&vergence);
				cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;

				char d;
				cout << "Specify desired position or reach to hand? [d|h]" << endl;
				cin >> d;

				double desiredGazeX, desiredGazeY, desiredVergence;
				if(d=='d')
				{
					cout << "Enter desired values for gazeX gazeY and vergence:" << endl;
					cin >> desiredGazeX >> desiredGazeY >> desiredVergence;
				}
				else
				{
					cout << "Which hand? [l/r]" << endl;
					char c;
					cin >> c;
					if(c=='r')
						rightHand=true;
					else
						rightHand=false;
				}

				cout << "Initialising tracker" << endl;
				tracker* t = new tracker(targetColour, ehCont);

				cout << "Starting tracker thread" << endl;
				boost::thread trackThrd(boost::bind(&tracker::track,t));
				//Use torso to bring target into reach

				//Use torso to bring target into reach
				if(d=='d')
					tor->LWPR_TorsoReach(desiredGazeX, desiredGazeY, desiredVergence);
				else
					tor->LWPR_TorsoReach(targetColour, rightHand);


				cout << "Waiting for torso-head move to finish" << endl;
				yarp::os::Time::delay(1.5);

				cout << "Interrupting tracker thread" << endl;
				trackThrd.interrupt();
				yarp::os::Time::delay(1.5);

				ehCont->fixate(targetColour);
				ehCont->verge(targetColour, &depth);

//				double gazeX, gazeY, vergence;
				ehCont->getGazeDirection(&gazeX, &gazeY);
				ehCont->getVergence(&vergence);
				cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;

				break;
			}

			case 12: //Get gaze point in world space
			{
				double x,y,z;
				getGazeCoordinates(&x,&y,&z);
				cout << "World gaze coordinates are: " << x << ", " << y << ", " << z << endl;

				double gazeX, gazeY, vergence;
				ehCont->getGazeDirection(&gazeX, &gazeY);
				ehCont->getVergence(&vergence);
				cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;
				break;
			}

			case 13: //Test centering of torso on target
			{
				double vergence;
				ehCont->getVergence(&vergence);
				tor->LWPR_centerTorsoOnTarget(vergence);
				break;
			}
//***************************************************************************************

			case exit:
				cout << "exiting" << endl;
				break;



//***************************************************************************************


			case 66: //Train gaze map to a set of reaches for schemas
			{
				std::string message;
				double dist;
				//x=-340
				for(double x=-340; x<=-260; x+=40)
				{
					for(double z=0; z<=280; z+=75)
					{
						//Left
						for(double y=-300; y<=0; y+=75)
						{
							cout << "Adding reach " << x/1000 << ", " << y/1000 << ", " << z/1000 << ", left" << endl;
							yarp::os::Bottle& b = portOut.prepare();
							b.clear();
							b.addString("target");
							b.addString("left_arm");
							b.addDouble(x/1000);
							b.addDouble(y/1000);
							b.addDouble(z/1000);
							b.addString("horizontal");
							portOut.write();

							do{
								yarp::os::Time::delay(0.3);
								reachResponse=portIn.read(true);
								message = reachResponse->get(0).asString().c_str();
								cout << "received: " << message << endl;
							}while(message.compare("complete")!=0);
							dist = reachResponse->get(1).asDouble();
							cout << "Reach distance: " << dist;

							bool success = getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
							if(!success)
							{
								ehCont->getHeadController()->move(20,-20,true);
								ehCont->getEyeController()->move(-10,-15, true);
								success = getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
								if(!success)
								{
									char c;
									cout << "Help me" << endl;
									cin >> c;
									getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
								}
							}

						}
						cout << "going to home" << endl;
						command("home");
						do{
							yarp::os::Time::delay(0.3);
							reachResponse=portIn.read(true);
							message = reachResponse->get(0).asString().c_str();
							cout << "received: " << message << endl;
						}while(message.compare("home")!=0);

						//right
						for(double y=0; y<=300; y+=80)
						{
							cout << "Adding reach " << x << ", " << y << ", " << z << ", right" << endl;
							yarp::os::Bottle& b = portOut.prepare();
							b.clear();
							b.addString("target");
							b.addString("right_arm");
							b.addDouble(x/1000);
							b.addDouble(y/1000);
							b.addDouble(z/1000);
							b.addString("horizontal");
							portOut.write();

							do{
								yarp::os::Time::delay(0.3);
								reachResponse=portIn.read(true);
								message = reachResponse->get(0).asString().c_str();
								cout << "received: " << message << endl;
							}while(message.compare("complete")!=0);
							dist = reachResponse->get(1).asDouble();
							cout << "Reach distance: " << dist;

							bool success = getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
							if(!success)
							{
								ehCont->getHeadController()->move(-20,-20,true);
								ehCont->getEyeController()->move(10,-15, true);
								success = getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
								if(!success)
								{
									char c;
									cout << "Help me" << endl;
									cin >> c;
									if(c=='y')
										getTargetCoordinates(handColour, &worldHandX, &worldHandY, &worldHandZ);
								}
							}
						}
						cout << "going to home" << endl;
						command("home");
						do{
							yarp::os::Time::delay(0.3);
							reachResponse=portIn.read(true);
							message = reachResponse->get(0).asString().c_str();
							cout << "received: " << message << endl;
						}while(message.compare("home")!=0);
					}
				}

				break;
			}


			case 99: //Torso-reach video
			{
				//Torso reach film tester
				yarp::os::Time::delay(2.0);
				command("home");
				bool last_arm = false;
				targetColour = "red";
				for(int i=0; i<100; i++)
				{
					yarp::os::Time::delay(2.0);
					ehCont->toRest();
					yarp::os::Time::delay(0.5);
					write(neu);
					yarp::os::Time::delay(1.0);


					gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);


					if(worldTargY>0)
						rightHand = true;
					else
						rightHand = false;

					if(last_arm!=rightHand)
						command("home", last_arm);


					bool torso = false;
					if(worldTargX <-340 || worldTargX >-260 ||
						worldTargY <-250 || worldTargY > 250 ||
						worldTargZ < 0 || worldTargZ > 240)
					{
						torso = true;
						//Use the torso to bring the target into reach.
						cout << "Initialising tracker" << endl;
						tracker* t = new tracker(targetColour, ehCont);

						cout << "Starting tracker thread" << endl;
						boost::thread trackThrd(boost::bind(&tracker::track,t));

						tor->LWPR_TorsoReach(targetColour, rightHand);

						cout << "Waiting for torso-head move to finish" << endl;
						yarp::os::Time::delay(0.5);

						cout << "Interrupting tracker thread" << endl;
						trackThrd.interrupt();
						yarp::os::Time::delay(1.0);

//						ehCont->fixate(targetColour);
//						ehCont->verge(targetColour, &depth);
						gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
					}


					gotHand=false;



					yarp::os::Bottle& b = portOut.prepare();
					b.clear();
					b.addString("target");
					if(worldTargY>0)
						b.addString("right_arm");
					else
						b.addString("left_arm");
					b.addDouble(worldTargX/1000);
					b.addDouble(worldTargY/1000);
					b.addDouble((worldTargZ+50)/1000);
					b.addString("horizontal");
					portOut.write();
					std::string message;
					double dist;
					do{
						yarp::os::Time::delay(0.3);
						reachResponse=portIn.read(true);
						message = reachResponse->get(0).asString().c_str();
						cout << "received: " << message << endl;
					}while(message.compare("complete")!=0 && message.compare("unreachable")!=0);

					if(message.compare("complete")==0)
					{
						dist = reachResponse->get(1).asDouble();
						cout << "Reach distance: " << dist;
						grippy->grasp(rightHand);
//						grippy->grasp(rightHand);
						bool success = grippy->grasp(rightHand);
						if(success)
							cout << "I think I've got it" << endl;
						else
							cout << "Hmmm, don't think that worked" << endl;
					}

					tor->toRest();
					command("home");

					if(message.compare("complete")==0)
					{
						yarp::os::Time::delay(2.0);

						b = portOut.prepare();
						b.clear();
						b.addString("target");
						if(rightHand)
						{
							b.addString("right_arm");
							b.addDouble(-0.28);
							b.addDouble(0.15);
							b.addDouble(0.12);
						}
						else
						{
							b.addString("left_arm");
							b.addDouble(-0.28);
							b.addDouble(-0.15);
							b.addDouble(0.12);
						}
					//	b.addDouble(offsetx/1000);
					//	b.addDouble(offsety/1000);
					//	b.addDouble(offsetz/1000);
	//					b.addDouble(-0.28);
	//					b.addDouble(0.15);
	//					b.addDouble(0.12);
						b.addString("horizontal");
						portOut.write(true);

						do{
							yarp::os::Time::delay(0.3);
							reachResponse=portIn.read(true);
							message = reachResponse->get(0).asString().c_str();
							cout << "received: " << message << endl;
						}while(message.compare("complete")!=0 && message.compare("unreachable")!=0);

						grippy->release(rightHand);
					}

					last_arm = rightHand;
					if(torso)
						write(hap2);
					else
						write(hap1);

					yarp::os::Time::delay(2.0);
				}
				break;
			}

			case 999: //Torso-reach gyda tracking for video
			{
				//Torso reach film tester
				ehCont->getEyeController()->verg(11,true);
				yarp::os::Time::delay(2.0);
				command("home");
				bool last_arm = true;
				rightHand = true;
				targetColour = "red";
				bool reach=true;
				while(reach)
				{
					bool unreachable = false;
					if(reachResponse=portIn.read(false))
					{
						std::string message = reachResponse->get(0).asString().c_str();
						if(message.compare("unreachable")==0)
						{
							write(sad);
							command("home", last_arm);
							//arms to home
							//use torso to move arm into range
							unreachable = true;
							torsoMove(worldTargX, worldTargY, worldTargZ);
						}
						else if(message.compare("complete")==0)
						{
							write(hap2);
							//torso to home
							command("home", last_arm);
							//head to "person"
							ehCont->getHeadController()->move(-15, 7, false);
							ehCont->getEyeController()->move(10,5,true);
							tor->toRest();
							yarp::os::Time::delay(1.5);

							if(!rightHand)
								break;
							else
							{
								write(neu);
								rightHand=false;
								last_arm = false;
								ehCont->getHeadController()->move(17, 13, false);
								ehCont->getEyeController()->move(-10,-15,true);
								yarp::os::Time::delay(0.5);
							}
						}
						else if(message.compare("waiting")==0)
						{
							write(neu);
						}
						else if(message.compare("reaching")==0)
						{
							write(hap1);
						}
					}
					gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ, false);
					yarp::os::Bottle& b = portOut.prepare();
					b.clear();
					b.addString("target");
					if(rightHand)
						b.addString("right_arm");
					else
						b.addString("left_arm");
					b.addDouble(worldTargX/1000);
					if(rightHand)
						b.addDouble((worldTargY-20)/1000);
					else
						b.addDouble((worldTargY+20)/1000);
					b.addDouble((worldTargZ+50)/1000);
					b.addString("horizontal");
					portOut.write(true);
				}

				break;
			}

			case 77: //Not finished yet
			{
				//Using Vam object and trying to make a fancy video
				float x,y;
				int xl,yl,xr,yr;
				vam->getRandomObject(&x,&y);
				ehCont->fixateEyeHead(x,y);
				ehCont->getEyeSaccader()->verge(vam);

				int id, noFeatures;
				vam->getFixatedObject(&id,&noFeatures);

				double wx,wy,wz, dist;
				getTargetCoordinates(&wx, &wy, &wz);
				bool success = reachToPoint(wx,wy,wz, &dist);
				if(!success)
				{
//					torsoMove(wx, wy, wz);		//Tracks targetColour

				}

				break;
			}

			case 128: //Learn eye+head+torso control
			{
//				ehCont->getHeadController()->move(0, -15, true);

				cout << "+++++ Starting eye learning +++++" << endl;
				int numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "Learnt eye saccades after " << numEyeSaccades << " attempts" << endl;
				ehCont->saveMaps();

				cout << "+++++ Starting head learning +++++" << endl;
				int numHeadSaccades = ehCont->learnEyeHeadSaccades();
				cout << "Learnt eye-head saccades after " << numHeadSaccades << " attempts, " <<
						"with a maximum set at 100" << endl;
				ehCont->saveMaps();

//				cout << "+++++ Starting torso learning +++++" << endl;
//				tor->LWPR_TorsoLearnerFullModel();

				break;
			}


			case 88: //Learn torso control
				cout << "+++++ Starting torso learning +++++" << endl;
				tor->LWPR_TorsoLearnerFullModel();
				break;

			case 55: //VAM saccading
			{
				float x,y;
				ehCont->getEyeController()->verg(11,true);
				double wx,wy,wz, dist;
				for(int i=0; i<10; i++)
				{
					ehCont->getEyeController()->move(0,0,false);
					ehCont->getHeadController()->move(0, -20, true);
					command("home");
					yarp::os::Time::delay(2.0);

					bool success = vam->getRandomObject(&x,&y);
					if(!success)
						continue;
					cout << "Received coordinates (" << x << ", " << y << ")" <<endl;
					ehCont->fixateEyeHead(x,y);
					getGazeCoordinates(&wx, &wy, &wz);
					reachToPoint(wx, wy, wz, &dist);
					int id, noFeatures;
					vam->getFixatedObject(&id, &noFeatures);

					yarp::os::Time::delay(1.0);
				}

				break;
			}

			case 44:	//hand tracker
			{
				tracker* t = new tracker();
				t->initTrack(ehCont->getMotorDriver(), target, ehCont);
				boost::thread trackThrd(boost::bind(&tracker::track,t));
				char c;
				cin >> c;
				trackThrd.interrupt();
				break;
			}

			case 110:
			{
				tor->closeMatlabPorts();
				break;
			}
			case 112:
			{
				tor->initMatlabPorts();
				break;
			}

			case 111: //Use the torso to bring the target into reach
			{
				cout << "Enter the model number for the log file name" << endl;
				string suffix;
				cin >> suffix;

//				string fullpath = path + "LWPR-Models/logs/extraTest_" + suffix + ".txt";
//				ofstream testExtraLog;
//				testExtraLog.open(fullpath.c_str(), ios::out | ios::app);	//append the contents onto the end of the file

				//torRot torTilt startgazex startgazey startvergence desiredgazex desiredgazey desiredVerg newtorRot newtortilt newgazex newgazy newvergence


				//******************Target 1***********************
				//Final motor coordinates should be: Tor(-10, -5) Head(17,1) Eye(0,0,15)
				//*Torso position 1:*
				cout << "********** Starting position 1 *********" << endl;
				bool success = torsoTest(suffix, -10, 10, -21, 0.7, 14.7);
				//*Torso position 2:*
				cout << "********** Starting position 2 *********" << endl;
				success = torsoTest(suffix, 15, -5, -21, 0.7, 14.7);
				//*Torso position 3:*
				cout << "********** Starting position 3 *********" << endl;
				success = torsoTest(suffix, 0, 20, -21, 0.7, 14.7);


				cout << "Move the target to Postion 2, and enter y when ready, or x to stop" << endl;
				char c;
				cin >> c;
				if(c!='y')
				{
//					testExtraLog.close();
					break;
				}
				//******************Target 2***********************
				//Final motor coordinates should be: Tor(0, 23) Head(-6,-9) Eye(1,2,9)
				//*Torso position 1:*
				cout << "********** Starting position 1 *********" << endl;
				success = torsoTest(suffix, -10, 10, 7.99, -8.7, 9.4);
				//*Torso position 2:*
				cout << "********** Starting position 2 *********" << endl;
				success = torsoTest(suffix, 15, -5, 7.99, -8.7, 9.4);
				//*Torso position 3:*
				cout << "********** Starting position 3 *********" << endl;
				success = torsoTest(suffix, 0, 20, 7.99, -8.7, 9.4);

				cout << "Move the target to Postion 3, and enter y when ready, or x to stop" << endl;
				cin >> c;
				if(c!='y')
				{
//					testExtraLog.close();
					break;
				}
				//******************Target 3***********************
				//Final motor coordinates should be: Tor(17, 13) Head(-11,-18) Eye(3,0,11)
				//*Torso position 1:*
				cout << "********** Starting position 1 *********" << endl;
				success = torsoTest(suffix, -10, 10, 24.2, -12.7, 11.1);
				//*Torso position 2:*
				cout << "********** Starting position 2 *********" << endl;
				success = torsoTest(suffix, 15, -5, 24.2, -12.7, 11.1);
				//*Torso position 3:*
				cout << "********** Starting position 3 *********" << endl;
				success = torsoTest(suffix, 0, 20, 24.2, -12.7, 11.1);

//				testExtraLog.close();

				break;
			}




			default:
				cout << "Unrecognised option, please try again" << endl;
				break;

		}
	}


#ifdef MONITOR
	monitorThrd.interrupt();
	m->closeLog();
#endif


	if(learn)
		saveAndQuit(0);
	else
		quit(0);
}

bool reachToPoint(double wx, double wy, double wz, double* dist)
{
	yarp::os::Bottle& b = portOut.prepare();
	b.clear();
	b.addString("target");
	if(wy>0)
		b.addString("right_arm");
	else
		b.addString("left_arm");
	b.addDouble(wx/1000);
	b.addDouble(wy/1000);
	b.addDouble((wz)/1000);
	b.addString("horizontal");
	portOut.write();

	std::string message;
	do{
		yarp::os::Time::delay(0.3);
		reachResponse=portIn.read(true);
		message = reachResponse->get(0).asString().c_str();
		cout << "received: " << message << endl;
	}while(message.compare("complete")!=0 && message.compare("unreachable")!=0);

	if(message.compare("complete")==0)
	{
		*dist = reachResponse->get(1).asDouble();
		cout << "Reach distance: " << *dist << endl;;
		return true;
	}
	else
		return false;
}


void torsoMove(double wx, double wy, double wz)
{
	cout << "Initialising tracker" << endl;
	tracker* t = new tracker(targetColour, ehCont);

	cout << "Starting tracker thread" << endl;
	boost::thread trackThrd(boost::bind(&tracker::track,t));

	bool rightHand;
	if(wy>0)
		rightHand = true;
	else
		rightHand = false;

	tor->LWPR_TorsoReach(targetColour, rightHand);

	cout << "Waiting for torso-head move to finish" << endl;
	yarp::os::Time::delay(0.5);

	cout << "Interrupting tracker thread" << endl;
	trackThrd.interrupt();
	yarp::os::Time::delay(1.0);
}

bool getGazeCoordinates(double *x, double *y, double *z)
{
	double torsoMotorConfig[3] = {0,0,0};
	double* headMotorConfig = new double[6];

	bool success = ehCont->getHeadController()->getCurrentPosition(headMotorConfig);
	cout << "Head motor config: ";
	for(int i=0; i<6; i++)
		cout << headMotorConfig[i] << " ";
	cout << endl;
	CalculateTargetWorldRef(torsoMotorConfig, headMotorConfig, x, y, z);	// returned in mm
	cout << "The gaze point in world space is: (" << *x << ", " << *y << ", " << *z << ")" << endl;
	return success;
}


bool getTargetCoordinates(string colour, double *x, double *y, double *z, bool check)
{
	double torsoMotorConfig[3] = {0,0,0};
	double* headMotorConfig = new double[6];

	double depth;	//vergence value

	bool success = ehCont->fixate(colour);

	if(success && check)
	{
		ehCont->verge(colour, &depth);
		ehCont->getHeadController()->getCurrentPosition(headMotorConfig);
		cout << "Head motor config: ";
		for(int i=0; i<6; i++)
			cout << headMotorConfig[i] << " ";
		cout << endl;
		CalculateTargetWorldRef(torsoMotorConfig, headMotorConfig, x, y, z);	// returned in mm
		cout << "The target position in world space is: (" << *x << ", " << *y << ", " << *z << ")" << endl;
	}
	else if(!check)
	{
		success = ehCont->getHeadController()->getCurrentPosition(headMotorConfig);
		cout << "Head motor config: ";
		for(int i=0; i<6; i++)
			cout << headMotorConfig[i] << " ";
		cout << endl;
		CalculateTargetWorldRef(torsoMotorConfig, headMotorConfig, x, y, z);	// returned in mm
	}
	return success;
}

bool getTargetCoordinates(double *x, double *y, double *z)
{
	double torsoMotorConfig[3] = {0,0,0};
	double* headMotorConfig = new double[6];

	double depth;	//vergence value

	bool success = ehCont->getHeadController()->getCurrentPosition(headMotorConfig);
	cout << "Head motor config: ";
	for(int i=0; i<6; i++)
		cout << headMotorConfig[i] << " ";
	cout << endl;
	CalculateTargetWorldRef(torsoMotorConfig, headMotorConfig, x, y, z);	// returned in mm
	cout << "The target position in world space is: (" << *x << ", " << *y << ", " << *z << ")" << endl;
	return success;
}

void command(string cmd)
{
	//Send command to reaching
	yarp::os::Bottle& b = portOut.prepare();
	b.clear();
	b.addString(cmd.c_str());
	portOut.write(true);
}

void command(string cmd, bool arm)
{
	//Send command to reaching
	yarp::os::Bottle& b = portOut.prepare();
	b.clear();
	b.addString(cmd.c_str());
	if(arm)
		b.addString("right_arm");
	else
		b.addString("left_arm");
	portOut.write(true);
}




bool torsoTest(string suffix, int torStartX, int torStartY,
		double desiredGazeX, double desiredGazeY, double desiredVergence)
{


	string fullpath = path + "LWPR-Models/logs/extraTest_" + suffix + ".txt";
				ofstream testExtraLog;
				testExtraLog.open(fullpath.c_str(), ios::out | ios::app);	//append the contents onto the end of the file


	//torRot torTilt startgazex startgazey startvergence desiredgazex desiredgazey desiredVerg newtorRot newtortilt newgazex newgazy newvergence

	double gazeX, gazeY, vergence, depth;
	double worldTargX, worldTargY, worldTargZ;
	double x,y;
	targetColour = "red";

	//*************Torso position 1:********************
	tor->getTorsoController()->move(torStartX, torStartY, true);
	tor->getTorsoController()->getCurrentPosition(&x, &y);
	testExtraLog << x << " " << y << " ";

	bool gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
	if(!gotTarg)
	{
		cout << "can you help me find the target [y/n]" << endl;
		char c;
		cin >> c;
		if(c!='y')
		{
			testExtraLog << " - " << endl;
			return false;
		}
		else
		{
			getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
		}
	}

	ehCont->getGazeDirection(&gazeX, &gazeY);
	ehCont->getVergence(&vergence);
	testExtraLog << gazeX << " " << gazeY << " " << vergence << " ";
	cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;

	testExtraLog << desiredGazeX << " " << desiredGazeY << " " << desiredVergence << " ";


	cout << "Initialising tracker" << endl;
	tracker* t1 = new tracker(targetColour, ehCont);

	cout << "Starting tracker thread" << endl;
	boost::thread trackThrd1(boost::bind(&tracker::track,t1));
	//Use torso to bring target into reach

	//Use torso to bring target into reach
	tor->LWPR_TorsoReach(desiredGazeX, desiredGazeY, desiredVergence);

	cout << "Waiting for torso-head move to finish" << endl;
	yarp::os::Time::delay(1.5);

	cout << "Interrupting tracker thread" << endl;
	trackThrd1.interrupt();
	yarp::os::Time::delay(1.5);

	gotTarg = ehCont->fixate(targetColour);
	tor->getTorsoController()->getCurrentPosition(&x, &y);
	testExtraLog << x << " " << y << " ";
	if(!gotTarg)
	{
		cout << "can you help me find the target [y/n]" << endl;
		char c;
		cin >> c;
		if(c!='y')
		{
			testExtraLog << " - " << endl;
			return false;
		}
		else
		{
			getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
		}
	}
	ehCont->verge(targetColour, &depth);

	ehCont->getGazeDirection(&gazeX, &gazeY);
	ehCont->getVergence(&vergence);
	cout << "Gaze direction: " << gazeX << ", " << gazeY << ", " << vergence << endl;


	testExtraLog << gazeX << " " << gazeY << " " << vergence << endl;

	testExtraLog.close();
	return true;
}

/*
 * demoVid.cpp
 *
 *  Created on: 6 May 2013
 *      Author: icub
 */
#include <signal.h>		//used in the save and quit functions

#include "EyeHeadController.h"
#include "handEyeCoordination.h"
#include "ShortTermMemoryV2.h"

#include "tracker.h"

#include "graspController.h"
#include "torsoSaccading.h"

EyeHeadSaccading* ehCont;
graspController* grippy;
Target* target;
torsoSaccading* tor;
var_params params;

yarp::os::BufferedPort<yarp::os::Bottle> portOut;
yarp::os::BufferedPort<yarp::os::Bottle> portIn;
yarp::os::Bottle* reachResponse;

/**
 * Attempts to get the world coordinates of an object w.r.t. the base of the torso.
 * Returns success of saccade.  Coordinates only set if successful, and are given in mm.
 */
bool getGazeCoordinates(double *x, double *y, double *z);
bool getTargetCoordinates(string colour, double *x, double *y, double *z);
void command(string cmd);

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



void quit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
	tor->closeMatlabPorts();

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
		cout << "A robot can be specified from the command line e.g. --robot [icub|icubSim]+F" << endl;
		params.m_ROBOT = "icub";
	}

	target = new Target();
	params.m_LOAD = true;

	if(options.check("path",val))
	{
		params.m_PATH = val->asString().c_str();
		params.m_LOAD = true;
		cout << "Loading files from path: " << params.m_PATH << endl;

		if(options.check("name",val))
		{
			params.m_FILENAME = val->asString().c_str();
			cout << "Loading file set: " << params.m_FILENAME << endl;
		}
		else
		{
			cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> params.m_FILENAME;
		}
	}
	else
	{

		cout << "Enter the path to the directory containing the files: e.g. ../data/ " <<endl;
		cin >> params.m_PATH;
		cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
		cin >> params.m_FILENAME;
	}

	target->initLog(params.m_PATH);
	bool learn = true;

	GazeMap* gm = new GazeMap();
	ehCont = new EyeHeadSaccading(gm, target);
	tor = new torsoSaccading(ehCont, target);

	armController* ac = new armController(true);
	grippy = new graspController(params.m_ROBOT, ac);

	yarp::os::Network yarp;

	portOut.open("/smc/reaching/out");
	portIn.open("/smc/reaching/in");
	emotePort.open("/emote/ask");

	yarp::os::Time::delay(0.5);
	yarp.connect("/smc/reaching/out", "/aber/reach/control:i");
	yarp.connect("/aber/reach/status:o", "/smc/reaching/in");
	yarp.connect("/emote/ask", "/emote/emotion");


	tor->initMatlabPorts();

	signal(SIGINT, quit);

	double targX, targY, gazeX, gazeY, depth;
	double worldTargX, worldTargY, worldTargZ;
	double worldHandX, worldHandY, worldHandZ;
	bool gotTarg=false, gotHand=false;
	bool rightHand=true;

	ehCont->getHeadController()->move(0, -25, true);

	cout << "Time to start filming..." << endl;
			//TIME TO START FILMING

char c;
cin >> c;

yarp::os::Time::delay(5);

	//***1. Locate target
	targetColour = "yellow";
	gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
	cout << "Target coordinates: " << worldTargX << ", " << worldTargY << ", " << worldTargZ << endl;

//cin >> c;
	//***2. Use torso to bring target into reach
	cout << "Initialising tracker" << endl;
	tracker* t = new tracker(targetColour, ehCont);

	cout << "Starting tracker thread" << endl;
	boost::thread trackThrd(boost::bind(&tracker::track,t));

	tor->LWPR_TorsoReach(targetColour, rightHand);

	cout << "Waiting for torso-head move to finish" << endl;
	yarp::os::Time::delay(1.5);

	cout << "Interrupting tracker thread" << endl;
	trackThrd.interrupt();
	yarp::os::Time::delay(1.5);

	gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
	cout << "Target coordinates: " << worldTargX << ", " << worldTargY << ", " << worldTargZ << endl;

//cin >> c;
	//***3. Reach the right arm towards the target
	cout << "Requesting reach towards target" << endl;
	double offsetx, offsety, offsetz;
	//Reach with right hand 10cm above object
	offsetx = -30;
	offsety = -100;
	offsetz = 100;

	offsetx += worldTargX;
	offsety += worldTargY;
	offsetz += worldTargZ;

	yarp::os::Bottle& b = portOut.prepare();
	b.clear();
	b.addString("target");
	b.addString("right_arm");
//	b.addDouble(offsetx/1000);
//	b.addDouble(offsety/1000);
//	b.addDouble(offsetz/1000);
	b.addDouble(-0.28);
	b.addDouble(0.15);
	b.addDouble(0.12);
	b.addString("horizontal");
	portOut.write(true);

	yarp::os::Time::delay(0.2);
	string message;
	cout << "Waiting for reach movement to complete" << endl;
	do{
		yarp::os::Time::delay(0.5);
		reachResponse = portIn.read(true);
		message = reachResponse->get(0).asString().c_str();
	}while(message.compare("complete")!=0);


//	offsetx = -30;
//	offsety = -100;
//	offsetz =  30;	//3 cm above target
//
//	offsetx += worldTargX;
//	offsety += worldTargY;
//	offsetz += worldTargZ;

//	b = portOut.prepare();
//	b.clear();
//	b.addString("target");
//	b.addString("right_arm");
//	b.addDouble(offsetx/1000);
//	b.addDouble(offsety/1000);
//	b.addDouble(offsetz/1000);
//	b.addString("horizontal");
//	portOut.write();
//
//	do{
//		yarp::os::Time::delay(0.5);
//		reachResponse = portIn.read(true);
//		message = reachResponse->get(0).asString().c_str();
//	}while(message.compare("complete")!=0);

//cin >> c;
	//***4. Grasp object
	grippy->grasp(rightHand);
	grippy->grasp(rightHand);

//cin >> c;
	//***5. Move right arm to joint grasp, vertical position

	cout << "Starting tracker thread" << endl;
	boost::thread trackThrd2(boost::bind(&tracker::track,t));

	b = portOut.prepare();
	b.clear();
	b.addString("target");
	b.addString("right_arm");
	b.addDouble(-0.30);
	b.addDouble(0.05);
	b.addDouble(0.1);
	b.addString("vertical");
	portOut.write();

	//***6. Straighten torso
	tor->toRest();
	ehCont->getHeadController()->move(10, -26, false);

	do{
		yarp::os::Time::delay(0.5);
		reachResponse = portIn.read(true);
		message = reachResponse->get(0).asString().c_str();
	}while(message.compare("complete")!=0);

	cout << "Waiting for arm-head move to finish" << endl;
	yarp::os::Time::delay(1.5);

	cout << "Interrupting tracker thread" << endl;
	trackThrd2.interrupt();
	yarp::os::Time::delay(1.5);


//cin >> c;
	//***7. Bring left hand to object
//	gotTarg = getTargetCoordinates(targetColour, &worldTargX, &worldTargY, &worldTargZ);
//	cout << "Target coordinates: " << worldTargX << ", " << worldTargY << ", " << worldTargZ << endl;

//	offsetx = -30;
//	offsety = -140;		//14 cm to left
//	offsetz =  0;
//
//	offsetx += worldTargX;
//	offsety += worldTargY;
//	offsetz += worldTargZ;

	b = portOut.prepare();
	b.clear();
	b.addString("target");
	b.addString("left_arm");
	b.addDouble(-0.28);
	b.addDouble(-0.10);
	b.addDouble(0.2);
	b.addString("vertical");
	portOut.write();

	do{
		yarp::os::Time::delay(0.5);
		reachResponse = portIn.read(true);
		message = reachResponse->get(0).asString().c_str();
	}while(message.compare("complete")!=0);

//cin >> c;
	offsetx = -30;
	offsety = -50;	//5 cm to left
	offsetz =  0;

	offsetx += worldTargX;
	offsety += worldTargY;
	offsetz += worldTargZ;

	b = portOut.prepare();
	b.clear();
	b.addString("target");
	b.addString("left_arm");
	b.addDouble(-0.28);
	b.addDouble(-0.06);
	b.addDouble(0.2);
	b.addString("vertical");
	portOut.write();

	do{
		yarp::os::Time::delay(0.5);
		reachResponse = portIn.read(true);
		message = reachResponse->get(0).asString().c_str();
	}while(message.compare("complete")!=0);

//cin >> c;
	//***8. Grasp object
	rightHand = false;
	grippy->grasp(rightHand);
	rightHand = true;
	grippy->release(rightHand);

//cin >> c;
	//Send right hand out away from object
	b = portOut.prepare();
	b.clear();
	b.addString("target");
	b.addString("right_arm");
	b.addDouble(-0.28);
	b.addDouble(0.15);
	b.addDouble(0.1);
	b.addString("vertical");
	portOut.write();


	write(hap2);
//	ehCont->toRest();
	ehCont->getHeadController()->move(10, -6, false);
	ehCont->getEyeController()->move(0, -3, false);
	ehCont->getEyeController()->verg(5, true);

	yarp::os::Time::delay(1.0);


	quit(0);
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


bool getTargetCoordinates(string colour, double *x, double *y, double *z)
{
	double torsoMotorConfig[3] = {0,0,0};
	double* headMotorConfig = new double[6];

	double depth;	//vergence value

	bool success = ehCont->fixate(colour);

	if(success)
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
	return success;
}

void command(string cmd)
{
	//Send command to reaching
	yarp::os::Bottle& b = portOut.prepare();
	b.clear();
	b.addString(cmd.c_str());
	portOut.write();
}


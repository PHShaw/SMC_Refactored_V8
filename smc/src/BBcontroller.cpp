/**
 *  BabyBot 2015: Attempting to reproduce an experiment performed by Claes von Hofsten in 1984.
 *  In this experiment, babies are observed every three weeks from 1 week old to 19 weeks old. Over this period,
 *  changes to their reaching behaviour are observed.  In particular, there is a significant reduction in the
 *  amount of reaching around week 7, and the behaviour of the hand, whether fisted or open also changes.
 *
 *  The aim of this program is to make use of the SMC and reaching software, with the addition of some updated
 *  vision processing that takes account of visual development, to simulate the learning over the first 19 weeks.
 *  Intermittent experiments are performed, with thresholds defined on the learning to limit the development in
 *  an attempt to match the ages at which the experiments are performed.
 *
 *  For more information, see the paper and associated notes on learning stages and experimental setup.
 *
 *  Created on: 3rd May, 2015
 *      Author: Patricia Shaw
 *      			Aberystwyth University
 *
 *
 *
 *  Learning mode:
 *  	In learning mode, it will learn the eye control, with the vision system gradually developing.
 *  	Thresholds are set based on learning performance, to define cut offs between learning sessions.
 *  	At each cut off, the current state of the mapping is saved with its stage name, before continuing.
 *
 *  	Note: This does not use the neck or the torso as part of this experiment
 *
 *  Experiment mode:
 *  	In this mode, the learnt maps are loaded and used to try and direct the eyes towards a specific
 *  	target moving back and forth in front of the iCub.  The opening and closing of the hand is also
 *  	controlled here.
 *
 *  	Two main control methods have been discussed:
 *  		1. Novelty control: Novelty and global excitation are used to decide what task to perform next.
 *  							This controls both the eye saccades, and the reaching.  As vision develops
 *  							and reaching gets repetitive over the early weeks, the number of saccades
 *  							increases whilst the number of reaches decreases.  Once the reaching evolves
 *  							into the next stage, learning
 *  		2. Muscle conflict: This refers specifically to the arm control. Rather than novelty of reflexive
 *  							movements decaying, the suggestion is a competing system gradually takes over.
 *  							During the transition, there is conflict between the two systems, reducing the
 *  							amount of reach movements performed.
 */


#include <signal.h>		//used in the save and quit functions

#include "EyeHeadController.h"

#include "tracker.h"

#include "graspController.h"
#include "monitor.h"
#include "Excitation.h"

/*
 * Various parameters are defined in params_config.h
 */

void setVisionParams(int acuity, int fov);
void setVisionModules();
void learningMode();
void experimentMode();

/**
 * Startup routine:
 * 0. create target
 * a. get path and filenames if needing to load up maps.
 * 3. create eye head controller
 * 4. obtain arm controller for controlling the hand
 */

EyeHeadSaccading* ehCont;
graspController* grippy;
Target* target;

using namespace smc;

yarp::os::BufferedPort<yarp::os::Bottle> portReachCommands;
yarp::os::BufferedPort<yarp::os::Bottle> portReachFeedback;
yarp::os::Bottle* reachResponse;

yarp::os::BufferedPort<yarp::os::Bottle> portVisionParameters;
yarp::os::BufferedPort<yarp::os::Bottle> portVisionModules;

void saveAndQuit(int param)
{
	target->closeLog();
	ehCont->closeLogs();

	ehCont->saveMaps();
	saveGazeMap();

	delete ehCont;

	std::exit(param);
}


//This function is used as above, but doesn't save the maps,
//i.e. when learning not enabled
void quit(int param)
{
	target->closeLog();
	ehCont->closeLogs();

	delete ehCont;

	std::exit(param);
}

int main(int argc, char* argv[])
{
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

	target = new Target();
	params.m_LOAD = false;

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

	ehCont = new EyeHeadSaccading(target);
	armController ac = new armController(true);	//grippy only.
	grippy = new graspController(ac);

	if(LEARN)
		signal(SIGINT, saveAndQuit);
	else
		signal(SIGINT, quit);

	yarp::os::Network yarp;

	portReachCommands.open("/smc/reaching/out");
	portReachFeedback.open("/smc/reaching/in");
//	yarp.connect("/smc/reaching/out", "/aber/reach/control:i");		//Move to a yarp application manager
//	yarp.connect("/aber/reach/status:o", "/smc/reaching/in");
	//Reach feedback connects to /aber/reach/BBfeedback:o


	portVisionParameters.open("/smc/vision/parameters/out");	//acuity / fov / brightness / threshold [string, int]
	//This connects to /target/parameter by yarp application manager.

	portVisionModules.open("/smc/vision/modules/out");

	char ready;
	cout << "Make the connections then press a key to continue" << endl;
	cin >> ready;


//now ready to start learning or performing experiments?

	//in learning mode, trigger learning with specified thresholds.  Save mappings in between.

	if(LEARN)
	{
		learningMode();
	}
	else //Experiment mode
	{
		//need to control eye, hand and arm using novelty excitations.
		experimentMode();
	}


}

/**
 * Send configuration parameters to vision module for acuity and field of view.
 *
 * @param acuity 	int 	odd number - this is the size of the matrix used to blur the image.
 * 											Larger numbers cause greater blurriness
 * @param fov 		int 	percentatge of adult (full) field of view
 *
 * TODO add variable for processing types i.e. colour, motion, shape and edge.  This can be
 * 			done with bitwise operations on an unsigned char.
 *
 */
void setVisionParams(int acuity, int fov)
{
	if(acuity % 2 == 0)
		acuity++;

	yarp::os::Bottle& b = portVisionParameters.prepare();
		b.clear();
		b.addString("acuity");
		b.addInt(acuity);	//accuity must be an odd number

		b.addString("fov");
		b.addInt(fov);	//percentage indicating FOV.
	portVisionParameters.write();
}

/**
 * This uses the params.visionFlags to turn relevant modules on and off.
 */
void setVisionModules()
{
	yarp::os::Bottle& b = portVisionModules.prepare();
		b.clear();
		b.addString("colour");
		if(params.m_VISION_FLAGS & COLOUR)
			b.addInt(1);	//enable
		else
			b.addInt(0);	//disable

		b.addString("shape");
		if(params.m_VISION_FLAGS & SHAPE)
			b.addInt(1);	//enable
		else
			b.addInt(0);	//disable

		b.addString("edge");
		if(params.m_VISION_FLAGS & EDGE)
			b.addInt(1);	//enable
		else
			b.addInt(0);	//disable

		b.addString("motion");
		if(params.m_VISION_FLAGS & MOTION)
			b.addInt(1);	//enable
		else
			b.addInt(0);	//disable

	portVisionModules.writer();
}


void learningMode()
{
	ehCont->setUseThresholds(true);
		string filename = params.m_FILENAME;


		//***********Weeks 0-1***********
		params.m_VISION_FLAGS |= COLOUR;
		setVisionModules();
		setVisionParams(21,30);
		ehCont->setEyeThreshold(4.0);
		int numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 0-1: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"1";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);

		//***********Weeks 1-4***********
		setVisionParams(17,45);
		ehCont->setEyeThreshold(2.0);
		numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 1-4: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"4";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);

		//***********Weeks 4-7***********
		setVisionParams(5,55);
		ehCont->setEyeThreshold(1.5);
		numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 4-7: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"7";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);

		//***********Weeks 7-10***********
		params.m_VISION_FLAGS |= MOTION;
		setVisionModules();
		setVisionParams(7,65);
		ehCont->setEyeThreshold(1.2);
		numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 7-10: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"10";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);

		//***********Weeks 10-13***********
		params.m_VISION_FLAGS |= EDGE;
		setVisionModules();
		setVisionParams(9,70);
		ehCont->setEyeThreshold(1.1);
		numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 10-13: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"13";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);

		//***********Weeks 13-16***********
		setVisionParams(11,80);
		ehCont->setEyeThreshold(1.1);
		numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 13-16: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"16";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);

		//***********Weeks 16-19***********
		params.m_VISION_FLAGS |= SHAPE;
		setVisionModules();
		setVisionParams(13,85);
		ehCont->setEyeThreshold(1.1);
		numEyeSaccades = ehCont->learnEyeSaccades();
		cout << "***********Weeks 16-19: Completed " << numEyeSaccades << " eye saccades***********" << endl;

		params.m_FILENAME = filename+"19";
		ehCont->saveMaps();

		yarp::os::Time::delay(1.0);
}



void experimentMode()
{
	Excitation novelty(0.1);

	LEARN = false;
	string filename = params.m_FILENAME;

	//connect to reaching
	//initialise hand //TODO add fist and spread to grippy options.

	//***********Weeks 0-1***********
		params.m_VISION_FLAGS |= COLOUR;
		setVisionModules();
		setVisionParams(21,30);

		//Load week 1 files
		//Using global excitation, and current max, decide on actions and frequency/delays between actions.
		params.m_FILENAME = filename+"1";





}

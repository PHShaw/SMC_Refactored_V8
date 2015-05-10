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
#
#include "tracker.h"
#include "ReachController.h"
#include "graspController.h"
#include "monitor.h"
#include "Excitation.h"

using namespace smc;

/*
 * Various parameters are defined in params_config.h
 */

void setVisionParams(int acuity, int fov);
void setVisionModules();
void learningMode(int stage = 1);
void experimentMode(int stage =1);
bool experimentConfiguration(string stageName, int acuity, int fov, int reachStage, float reachSaturation);
bool getGazeCoordinates(double *x, double *y, double *z);
void bbExperiment(int reachStage);

/**
 * Startup routine:
 * 0. create target
 * a. get path and filenames if needing to load up maps.
 * 3. create eye head controller
 * 4. obtain arm controller for controlling the hand
 */

EyeHeadSaccading* ehCont;
ReachController reachCont;
graspController* grippy;
Target* target;
Excitation novelty(0.1);
var_params params;

const int acuityLevels[7]=
//{   40 ,   35  ,   25  ,  20	,	15	,	 10	,	 5 };	//basic vision
{   80 ,   70  ,   50  ,  40	,	30	,	 20	,	 10 };
//week
//   1		4		7	  10	   13		16	   19
const int fovLevels[7]=
{   50 ,   60  ,   65  ,  70	,	75	,	 80	,	85 };



yarp::os::BufferedPort<yarp::os::Bottle> portVisionParameters;
yarp::os::BufferedPort<yarp::os::Bottle> portVisionModules;

void initParams(int argc, char* argv[])
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
}

void initControllers()
{
	target->initLog(params.m_PATH);


	ehCont = new EyeHeadSaccading(target);
	armController* ac = new armController(true);	//grippy only.
	grippy = new graspController(params.m_ROBOT, ac, false);
}

void configurePorts()
{
	yarp::os::Network yarp;

	reachCont.openPorts();

	portVisionParameters.open("/smc/vision/parameters/out");	//acuity / fov / brightness / threshold [string, int]
	//This connects to /target/parameter by yarp application manager.

	portVisionModules.open("/smc/vision/modules/out");

	char ready;
	cout << "Make the connections then press a key to continue" << endl;
	cin >> ready;
}

void closePorts()
{
	portVisionParameters.close();
	portVisionModules.close();

	reachCont.closePorts();
	grippy->closePorts();
}


void saveAndQuit(int param)
{
	closePorts();
	target->closeLog();
	ehCont->closeLogs();

	ehCont->saveMaps();

	delete ehCont;

	std::exit(param);
}


//This function is used as above, but doesn't save the maps,
//i.e. when learning not enabled
void quit(int param)
{
	closePorts();
	target->closeLog();
	ehCont->closeLogs();

	delete ehCont;

	std::exit(param);
}

int main(int argc, char* argv[])
{
	cout << "Learning or experimental mode (default=learning) [l|e]" << endl;
	char c;
	cin >> c;
	if(c == 'e')
		params.LEARN = false;
	else
		params.LEARN = true;


	cout << "Select stage [1-7]" << endl;
	int stage;
	cin >> stage;
	if(stage<1)
		stage = 1;
	else if(stage>7)
		stage = 7;


	initParams(argc, argv);

	initControllers();

	configurePorts();

	if(params.LEARN)
		signal(SIGINT, saveAndQuit);
	else
		signal(SIGINT, quit);


	//move head to starting position
	ehCont->getEyeController()->verg(10,true);
	ehCont->getHeadController()->move(0,-15, true);


//now ready to start learning or performing experiments?

	//in learning mode, trigger learning with specified thresholds.  Save mappings in between.
	if(params.LEARN)
	{
		learningMode(stage);
	}
	else //Experiment mode
	{
		//need to control eye, hand and arm using novelty excitations.
		experimentMode(stage);
	}


}

/**
 * Send configuration parameters to vision module for acuity and field of view.
 *
 * @param acuity 	int 	odd number - this is the size of the matrix used to blur the image.
 * 											Larger numbers cause greater blurriness
 * @param fov 		int 	percentatge of adult (full) field of view
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

	portVisionModules.write();
}


void learningMode(int stage)
{


	ehCont->setUseThresholds(true);
		string filename = params.m_FILENAME;

		int acuity,fov;	//Acuity has a range from 1 to 199, and must be odd.
						//TODO The numbers used will need to be tuned once vision is running on the iCub.
		int numEyeSaccades =0;
			switch(stage){
				//Note, this will auto progress onto the next stage;
			case 1:
				//***********Weeks 0-1***********
				params.m_VISION_FLAGS |= COLOUR;
				setVisionModules();
				acuity = acuityLevels[0];
				fov = fovLevels[0];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(4.0);

				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 0-1: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"1";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);


			case 2:
				//***********Weeks 1-4***********
				params.m_VISION_FLAGS |= COLOUR;
				acuity=acuityLevels[1];
				fov=fovLevels[1];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(2.0);
				ehCont->getEyeSaccader()->resetStats();
				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 1-4: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"4";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);

			case 3:
				//***********Weeks 4-7***********
				params.m_VISION_FLAGS |= COLOUR;
				acuity=acuityLevels[2];
				fov=fovLevels[2];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(1.5);
				ehCont->getEyeSaccader()->resetStats();
				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 4-7: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"7";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);

			case 4:
				//***********Weeks 7-10***********
				params.m_VISION_FLAGS |= COLOUR;
				params.m_VISION_FLAGS |= MOTION;
				setVisionModules();
				acuity=acuityLevels[3];
				fov=fovLevels[3];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(1.2);
				ehCont->getEyeSaccader()->resetStats();
				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 7-10: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"10";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);

			case 5:
				//***********Weeks 10-13***********
				params.m_VISION_FLAGS |= COLOUR;
				params.m_VISION_FLAGS |= MOTION;
				params.m_VISION_FLAGS |= EDGE;
				setVisionModules();
				acuity=acuityLevels[4];
				fov=fovLevels[4];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(1.1);
				ehCont->getEyeSaccader()->resetStats();
				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 10-13: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"13";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);

			case 6:
				//***********Weeks 13-16***********
				params.m_VISION_FLAGS |= COLOUR;
				params.m_VISION_FLAGS |= MOTION;
				params.m_VISION_FLAGS |= EDGE;
				acuity=acuityLevels[5];
				fov=fovLevels[5];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(1.1);
				ehCont->getEyeSaccader()->resetStats();
				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 13-16: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"16";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);

			case 7:
				//***********Weeks 16-19***********
				params.m_VISION_FLAGS |= COLOUR;
				params.m_VISION_FLAGS |= MOTION;
				params.m_VISION_FLAGS |= EDGE;
				params.m_VISION_FLAGS |= SHAPE;
				setVisionModules();
				acuity=acuityLevels[6];
				fov=fovLevels[6];
				setVisionParams(acuity,fov);
				ehCont->setEyeThreshold(1.1);
				ehCont->getEyeSaccader()->resetStats();
				numEyeSaccades = ehCont->learnEyeSaccades();
				cout << "***********Weeks 16-19: Completed " << numEyeSaccades << " eye saccades***********" << endl;

				params.m_FILENAME = filename+"19";
				ehCont->saveMaps();
				stage++;
				yarp::os::Time::delay(1.0);

			default:
				cout <<"Finished babybot eye learning stages" << endl;
				break;
			}//end of switch
}



void experimentMode(int stage)
{


	int acuity,fov;
	int reachStage;
	float reachSaturation;


	params.LEARN = false;
//	string filename = params.m_FILENAME;
	char c;

	//connect to reaching

	switch(stage){

	case 1:
	{
	//***********Weeks 0-1***********
		params.m_VISION_FLAGS |= COLOUR;
		setVisionModules();
		acuity = acuityLevels[0];
		fov = fovLevels[0];
		setVisionParams(acuity,fov);


		reachStage=0;
		reachSaturation=0.14;	//predefined saturation threshold for learning.
								//Ideally this should come from learning, integrated with vision control.

		//Load week 1 eye mapping files
//		filename = params.m_FILENAME+"1";
//		ehCont->loadFile(filename);
//		ehCont->getEyeSaccader()->setMaps(ehCont->getEyeMap());
		int links = ehCont->getEyeMap()->getNumGoodLinks();	//good coverage typically around 500 links.
		float saturation = links/500 * 100.0;
		novelty.setEyeExcitation((int)saturation);

		//Set up the current levels of excitation.
		novelty.setFovealExcitation(acuity,fov);
		novelty.setRetinaExcitation();	//takes parameters from params.m_vision_flags.
		novelty.setReachExcitation(reachStage, reachSaturation);

		bbExperiment(reachStage);

		cout << "Finished stage 0-1" << endl;
//		yarp::os::Time::delay(10);

		cout << "Ready to move to stage 1-4 (2), press any key" << endl;
		cin >> c;

	}
		//Using global excitation, and current max, decide on actions and frequency/delays between actions.

	//***********Weeks 1-4***********
	case 2:
		params.m_VISION_FLAGS |= COLOUR;
		acuity = acuityLevels[1];
		fov = fovLevels[1];
		reachStage = 0;
		reachSaturation = 0.57;

		experimentConfiguration("4", acuity, fov, reachStage, reachSaturation);
		bbExperiment(reachStage);

		cout << "Finished stage 1-4" << endl;
		//yarp::os::Time::delay(10);

		cout << "Ready to move to stage 4-7 (3), press any key" << endl;
		cin >> c;

	//***********Weeks 4-7***********
	case 3:
		params.m_VISION_FLAGS |= COLOUR;
		acuity=acuityLevels[2];
		fov=fovLevels[2];
		reachStage = 0;
		reachSaturation = 0.9;

		experimentConfiguration("4", acuity, fov, reachStage, reachSaturation);
		bbExperiment(reachStage);

		cout << "Finished stage 4-7" << endl;
//		yarp::os::Time::delay(10);

		cout << "Read to move to stage 7-10 (4), press any key" << endl;
		cin >> c;

	//***********Weeks 7-10***********
	case 4:
		params.m_VISION_FLAGS |= COLOUR;
		params.m_VISION_FLAGS |= MOTION;
		setVisionModules();
		acuity=acuityLevels[3];
		fov=fovLevels[3];
		reachStage = 1;
		reachSaturation = 0.33;

		experimentConfiguration("10", acuity, fov, reachStage, reachSaturation);
		bbExperiment(reachStage);

		cout << "Finished stage 7-10" << endl;
//		yarp::os::Time::delay(10);

		cout << "Ready to move to stage 10-13 (5), press any key" << endl;
		cin >> c;

	//***********Weeks 10-13***********
	case 5:
		params.m_VISION_FLAGS |= COLOUR;
		params.m_VISION_FLAGS |= MOTION;
		params.m_VISION_FLAGS |= EDGE;
		acuity=acuityLevels[4];
		fov=fovLevels[4];
		reachStage = 1;
		reachSaturation = 0.66;

		experimentConfiguration("13", acuity, fov, reachStage, reachSaturation);
		bbExperiment(reachStage);

		cout << "Finished stage 10-13" << endl;
//		yarp::os::Time::delay(10);

		cout << "Ready to move to stage 13-16 (6), press any key" << endl;
		cin >> c;

	//***********Weeks 13-16***********
	case 6:
		params.m_VISION_FLAGS |= COLOUR;
		params.m_VISION_FLAGS |= MOTION;
		params.m_VISION_FLAGS |= EDGE;
		acuity=acuityLevels[5];
		fov=fovLevels[5];
		reachStage = 1;
		reachSaturation = 0.9;

		experimentConfiguration("16", acuity, fov, reachStage, reachSaturation);
		bbExperiment(reachStage);

		cout << "Finished stage 10-16" << endl;
//		yarp::os::Time::delay(10);

		cout << "Ready to move to stage 16-19 (7), press any key" << endl;
		cin >> c;

	//***********Weeks 16-19***********
	case 7:
		params.m_VISION_FLAGS |= COLOUR;
		params.m_VISION_FLAGS |= MOTION;
		params.m_VISION_FLAGS |= EDGE;
		params.m_VISION_FLAGS |= SHAPE;
		acuity=acuityLevels[6];
		fov=fovLevels[6];
		reachStage = 2;
		reachSaturation = 0.33;

		experimentConfiguration("19", acuity, fov, reachStage, reachSaturation);
		bbExperiment(reachStage);

		cout << "Finished stage 16-19" << endl;
		yarp::os::Time::delay(10);

	default:
		cout << "Completed all stages" << endl;
		break;
	}
}


bool experimentConfiguration(string stageName, int acuity, int fov, int reachStage, float reachSaturation)
{
	float eyeLinkSaturationPoint = 500.0;

	setVisionModules();
	setVisionParams(acuity,fov);

	//Load week X eye mapping files
	string filename = params.m_FILENAME+stageName;
	bool success = ehCont->reloadMaps(filename);

	ehCont->getEyeSaccader()->resetStats();
	int links = ehCont->getEyeMap()->getNumGoodLinks();	//good coverage typically around 500 links.
	float saturation = links/eyeLinkSaturationPoint * 100.0;
	novelty.setEyeExcitation((int)saturation);
	novelty.setReachExcitation(reachStage, reachSaturation);

	//Update up the current levels of excitation.
	novelty.updateFovealExcitation(acuity,fov);
	novelty.updateRetinaExcitation();	//takes parameters from params.m_vision_flags.


	return true;
}


void bbExperiment(int reachStage)
{
	//Get global excitation [0..1] decide on activity level
	//Get most excited subsystem, and perform action using that subsystem
	//Decay?

	//Retina, fovea, eye -> eye saccade
	//Arm, hand -> reach

	//duration of experiment.
	time_t experimentDuration = 150;
	//2* blocks (1 fast (30s), 1 slow (60s), 60s gap between blocks, total time 2:30s)

	time_t startSeconds;
	startSeconds = time(NULL);
	int counter = 0;
	time_t timeTaken = startSeconds-startSeconds;
	while(timeTaken < experimentDuration)
	{
		double targX, targY;
		target->getTarget(&targX, &targY);
		int colourTargs = target->getNumTargets("colour");
		int motionTargs = target->getNumTargets("motion");

		novelty.updateRetinaExcitation(colourTargs,motionTargs);

		float globalEx = novelty.getGlobalExcitation();		//returns a number between 0 and 1.
		printf("Global Excitation: %f.2\n", globalEx);
		novelty.printExcitations();


		System excited = novelty.getMaxExcitation();
		cout << "Most excited subsystem: " << excited << endl;

		//Testing specific system
//		System excited = EYE;

		int inactivityCounter = 0;	//count number of sequential cases of inactivity;
		int activityLevel = randGenerator(10);
		printf("Activation level: %f.2\n", ((float)activityLevel/10.0));
		if(activityLevel<=(globalEx*10))
		{
			inactivityCounter=0;	// break in inactivity, so reset counter;
			//perform an action
			switch (excited){
				case FOVEAL:
					novelty.decay(FOVEAL);
				case RETINA:
					//TODO check for how many stimuli are currently visible.
				case EYE:
				{
					bool success = ehCont->fixate();	//TODO: Maybe this should be multi-threaded?
					int steps = ehCont->getEyeSaccader()->getStepCounter();
						//update novelty.
						//if not managed to fixate, novelty should be going down.
						//but likely to be a large number of steps as well, before failure.
					novelty.updateEyeExcitation(steps, success);
					break;
				}
				case ARM:
				case smc::HAND:
				{
					// if excitation for hand is high, then want to open hand, otherwise hand should be fisted
					//need to take into account reach stage, to select reach target.  In later stages, this will
					//be preceded by a saccade to get the location of the visual target.

					if(reachCont.getCurrentStatus()==REACHING)
					{
						bool okayReaching = reachCont.isReachingOkay();
						//TODO Decide what to do here?
					}

					if(reachStage==0)
					{
						/*
						 * Stage 0, reflex mode:
						 * left arm: -0.28 -0.13 0.13 horizontal (accuracy 0.077) ?
						 * right arm: -0.28 0.13 0.33 horizontal (accuracy 0.123) ?
						 */
						int arm = randGenerator(2);	//0 or 1
						if(arm == 0)
							reachCont.sendArmTarget(-0.35, -0.25, 0.30, false); //y -0.25 -> -0.05
						else
							reachCont.sendArmTarget(-0.34, 0.05, 0.27, true);

						float hand = novelty.getExcitation(smc::HAND);
						activityLevel = randGenerator(10);
						if(activityLevel<=(hand*10))
						{
							cout << "opening hand" << endl;
							grippy->release(arm);
						}
						else
						{
							cout << "closing hand" << endl;
							grippy->fist(arm);
						}

					}
					else// if(reachStage==1 || reachStage==2)
					{
						//vision should be starting to get involved.
						bool success = ehCont->fixate();
						int steps = ehCont->getEyeSaccader()->getStepCounter();
						novelty.updateEyeExcitation(steps, success);
						double x,y,z;
						getGazeCoordinates(&x, &y, &z);
						//x negative is forward
						//can go up to -0.20
						x /= 1000;
						if(reachStage==1 && x < -0.2)
							x = -0.2;
						else if(reachStage==2 && x < -0.35)
							x = -0.35;
						y /= 1000;
						z /= 1000;
						bool arm;
						if(y>0){
							arm = true;
							reachCont.sendArmTarget(x,y,z, arm);
						}
						else
						{
							arm = false;
							reachCont.sendArmTarget(x,y,z, arm);
						}

						float hand = novelty.getExcitation(smc::HAND);
						activityLevel = randGenerator(10);
						if(activityLevel<=(hand*10))
						{
							cout << "opening hand" << endl;
							grippy->release(arm);
						}
						else
						{
							cout << "closing hand" << endl;
							grippy->fist(arm);
						}

					}
					//Get feedback from reaching
					float dist;
					if(reachCont.getCurrentStatus()==COMPLETE)
					{
						dist = reachCont.getDistance();
						novelty.updateReachExcitation(dist);

						yarp::os::Time::delay(0.5);
						//Return hand to home position;
						reachCont.command("home");
						grippy->fist(true);
						grippy->fist(false);
					}
					else if(reachCont.getCurrentStatus()==REACHING)
					{
						dist = reachCont.getDistance();
						if(reachCont.isReachingOkay())
							novelty.updateReachExcitation(dist);
					}


					//TODO: Tuning, do we to leave the arm there (randomly)?
					//TODO: What should be done with the hand here?

					break;
				}
				default:
					break;
			}

		}//if activity level
		else{
			cout << "Not excited enough to move" << endl;
			inactivityCounter++;
		}
		if(inactivityCounter>=10)
		{
			novelty.stimulateSystem();
			inactivityCounter=0;
		}

		yarp::os::Time::delay(1);	//wait a second
		time_t current = time(NULL);
		timeTaken=current - startSeconds;
	}
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

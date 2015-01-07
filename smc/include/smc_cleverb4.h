/*
 * smc_cleverb4.h
 *
 * The aim of this class is to provide the real interface between the motor controller
 * and the DMc for use with the board experiments
 *
 *
 *  Created on: 19 Apr 2011
 *  Version 3: 27th Feb 2012
 *      Author: Patricia Shaw
 *      		Aberystwyth University
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include <fstream>
#include <iostream>
#include <list>
#include <signal.h>		//used in the save and quit functions

#include "FieldFieldMapping.h"
#include "FFM_IO.h"
#include "GazeMap.h"
#include "Gaze_IO.h"


#include "EyeHeadController.h"
#include "handEyeCoordination.h"
#include "ShortTermMemoryV2.h"

#include "common_cleverb4.h"


typedef FieldFieldMapping ffm;

//Step 1: Initialisation
//			a: Load in the learnt maps
//			b: Initialise yarp and iCub
//			c: Move iCub to starting/rest positions
//			d: Connect to DMc and VAM
//			e: Tell DMc we are now ready for commands

//Step 2: Experiment
//	We should now be set up and ready to communicate with the DMc
//			a: Wait for command from DMc
//			b: Act on command from DMc
//			c: when action completed, send message to DMc, return to 2a.



struct dmcCoord{
	dmcCoord(int x, int y, GazeField* f, string col)
	{
		dmcX = x;
		dmcY = y;
		gf = f;
		colour = col;
	}
	int dmcX, dmcY;
	GazeField* gf;
	std::string colour;
};


bool initYarp();
bool init();
void saveAndQuit(int param);


//DMC communication:
yarp::os::BufferedPort < yarp::os::Bottle > DMCinPort;
yarp::os::BufferedPort < yarp::os::Bottle > DMCoutPort;
yarp::os::Bottle* dmcCmd;

//VAM communication:
yarp::os::BufferedPort<yarp::os::Bottle> VAMinPort;
yarp::os::BufferedPort<yarp::os::Bottle> VAMoutPort;
yarp::os::Bottle* vamData;


EyeHeadSaccading* ehCont;
handEyeCoordination* heCoor;
armReaching* armReach;
ShortTermMemory* stm;

Target* target;
int imageWidth = 320;
int imageHeight = 240;

std::string path, filename;
bool board = true;
//bool learn = true;
bool safe = true;
const bool VAM = true;

const bool MSG_WAIT = true;

std::vector<dmcCoord*> convMem;


//int eye_position, arm_position;
//bool RIGHT_ARM = true;
//bool LEFT_ARM = false;

bool AU = false;
bool blind = false;
//bool synchronous = false;
//bool nearestNeighbour = true;



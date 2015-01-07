/*
 * common_cleverb4.h
 *
 *	This file contains the constant values used in messages exchanged between the
 *	SMc and the DMc.
 *  Created on: 27 Jan 2011
 *      Author: Patricia Shaw
 *      		Aberystwyth University
 */

#ifndef SMC_DMC_STUB_CONSTANTS
	#define SMC_DMC_STUB_CONSTANTS


//****************************
//* SMC TO DMC STATE MESSAGES
//****************************
const int UNKNOWN =0;




//*****************************
//* DMC TO SMC ACTION MESSAGES
//*****************************
const int DO_NOTHING=0;


//****************
// ARM ACTION MESSAGES
//****************
const int ARM_REACH_AND_PRESS=1;			// was just reach, is now reach and press
const int ARM_REACH_AND_PUSH_LEFT=2;		//THIS MAY NOT BE POSSIBLE, USES RIGHT ARM TO MAKE A LEFT PUSH
const int ARM_REACH_AND_PUSH_RIGHT=3;		//THIS MAY NOT BE POSSIBLE, USES LEFT ARM TO MAKE A RIGHT PUSH
const int ARM_GO_HOME=4;



//****************
// ARM STATE MESSAGES
//****************
const int ARM_ACTION_COMPLETE=1;			// Successfully completed arm movement
const int ARM_MOVING=2;						// Arm is currently moving
const int ARM_FAILED=3;						// Arm action could not be completed, due to range.


//******************************************
// String equivalents of the numeric values
//******************************************


const char* armActionStrings[]=
	{ "Do Nothing",
	  "Arm reach and press where eye is",
	  "Arm reach and push left where eye is",
	  "Arm reach and push right where eye is",
	  "Arm go home"
	};

#endif

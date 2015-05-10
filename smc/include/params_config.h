/*
 * params_config.h
 *
 *  Created on: 9 Nov 2012
 *      Author: icub
 */


#ifndef SMC_PARAMETER_CONFIGURATION
	#define SMC_PARAMETER_CONFIGURATION

#include <string>

//std::string static robot;
//std::string static robot = "icub";
//std::string static robot = "icubF";		//preferred
//std::string static robot = "icubSim";
//std::string static robot = "icubSimF";


 bool const SYNCHRONOUS = false;
 bool const NEAREST_NEIGHBOUR = true;
 bool const RANDOM_MOVES=true;
 bool const SAFEMODE = true;
 bool const BASIC_VISION = false;

 float const RETINA_WIDTH = 320.f;	// OR 320
 float const RETINA_HEIGHT = 240.f; 	// OR 240

 unsigned char const COLOUR = 0x01;
 unsigned char const MOTION = 0x02;
 unsigned char const EDGE   = 0x03;
 unsigned char const SHAPE  = 0x04;

/*
 * to turn on bits:
 *   unsigned char myflags = 0x00;
 *   myflags |= COLOUR;
 *   myflags |= MOTION;
 *
 * to turn off bits:
 *   myflags &= ~EDGES;
 *
 * to toggle options:
 *   myflags ^= SHAPE;
 *
 * to query bit states:
 *   if(myflags & MOTION) ...//motion is set.
 */


struct var_params{
	bool LEARN;
	std::string m_FILENAME;// = "testXV10";
	std::string m_PATH;//="../../data/";
	bool m_LOAD;//=true;
	std::string m_ROBOT;// = "icubSim";



	unsigned char m_VISION_FLAGS; //=COLOUR;

}extern params;


#endif


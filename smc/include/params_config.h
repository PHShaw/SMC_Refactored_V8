/*
 * params_config.h
 *
 *  Created on: 9 Nov 2012
 *      Author: icub
 */


#ifndef SMC_PARAMETER_CONFIGURATION
	#define SMC_PARAMETER_CONFIGURATION

//std::string static robot;
//std::string static robot = "icub";
//std::string static robot = "icubF";		//preferred
//std::string static robot = "icubSim";
//std::string static robot = "icubSimF";


bool const LEARN = true;
bool const SYNCHRONOUS = false;
bool const NEAREST_NEIGHBOUR = true;
bool const SAFEMODE = true;


struct var_params{
	std::string m_FILENAME = "testXV10";
	std::string m_PATH="../../data/";
	bool m_LOAD=true;
	std::string m_ROBOT = "icubSim";
}params;


#endif


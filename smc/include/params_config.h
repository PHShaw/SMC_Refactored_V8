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


bool const learn = true;
bool const synchronous = false;
bool const nearestNeighbour = true;
bool const safeMode = true;


struct var_params{
	std::string filename = "testXV10";
	std::string path="../../data/";
	bool load=true;
	std::string robot = "icubSim";
}params;


#endif


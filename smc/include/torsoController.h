/*
 * torsoController.h
 *
 *  Created on: 10 Nov 2011
 *      Author: icub
 */

#ifndef TORSO_CONTROLLER
	#define TORSO_CONTROLLER

#include <iostream>
#include <fstream>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "utilities.h"




class torsoController{
public:
	torsoController(std::string robot);
	bool initYarp(std::string robot);
	bool stationary();
	bool move(const double x, const double y, bool block=true);
	void generateRandomPosition(double* x, double* y);
	void generateSmallRelativeMovement(int maxSize, double* x, double* y);
	void generateRelativeMovement(int xmaxSize, int ymaxSize, double* x, double* y);
	void babble(bool block=true);
	void toRest();
	bool getCurrentPosition(double* x, double* y);
	bool getCurrentPosition(double* headMotorPos);
	bool isAtPosition(const double x, const double y);	//compares to current position

	//botch functions
	yarp::dev::PolyDriver* getMotorDriver(){return motordriver;}

private:
//	bool init();
	yarp::dev::PolyDriver *motordriver;
	yarp::dev::IPositionControl *pos;
	yarp::dev::IEncoders *enc;

	std::ofstream motorlogfile;


	//Joint IDs
	const static int torsoPitch = 2;	//y
	const static int torsoYaw 	= 0;	//x
	const static int torsoRoll  = 1;

	//Movement ranges
	const static double xMin = -40.0;
	const static double xMax =  40.0;
	const static double yMin = -20.0;
	const static double yMax =  35.0;
};

#endif

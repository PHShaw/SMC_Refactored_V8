/*
 * eyeController.h
 *
 *  Created on: 8 Aug 2011
 *      Author: icub
 */

#ifndef EYE_CONTROLLER
	#define EYE_CONTROLLER

#include <iostream>

#include <yarp/os/all.h>	//Time
#include <yarp/dev/all.h>

#include "utilities.h"



class eyeController{
public:
	eyeController(yarp::dev::PolyDriver* pmotordriver);
	bool stationary();
	void move(const double x, const double y, bool block=true);
	void verg(const double vergence, bool block=true);
	void generateRandomPosition(double* x, double* y);
	void generateSmallRelativeMovement(double* x, double* y, double maxRange);
	void babble(bool block=true);
	void smallbabble(bool block=true, double maxRange=5);
	void toRest();
	bool getCurrentPosition(double* x, double* y);
	bool getCurrentPosition(double* headMotorPos);
	bool getVergence(double* vergence);
	bool isAtPosition(const double x, const double y);	//compares to current position

	//Movement ranges
	const static double xMin = -30.0;
	const static double xMax =  30.0;
	const static double yMin = -35.0;
	const static double yMax =  18.0;

private:
//	bool init();
	yarp::dev::PolyDriver *motordriver;
	yarp::dev::IPositionControl *pos;
	yarp::dev::IEncoders *enc;

	//Joint IDs
	const static int eyeX = 4;
	const static int eyeY = 3;


};

#endif

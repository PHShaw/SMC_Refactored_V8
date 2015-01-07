/*
 * headController.h
 *
 *  Created on: 8 Aug 2011
 *      Author: icub
 */


#include <iostream>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "utilities.h"



class headController{
public:
	headController(yarp::dev::PolyDriver* pmotordriver);
	bool stationary();
	void move(const double x, const double y, bool block=true);
	void generateRandomPosition(double* x, double* y);
	void generateSmallRelativeMovement(double* x, double* y, double maxRange);
	void babble(bool block=true);
	void smallbabble(bool block=true, double maxRange=5);
	void toRest();
	void headDown();
	bool getCurrentPosition(double* x, double* y);
	bool getCurrentPosition(double* headMotorPos);
	bool isAtPosition(const double x, const double y);	//compares to current position

	const static double xMin = -42.0;
	const static double xMax =  42.0;
	const static double yMin = -35.0;
	const static double yMax =  18.0;


private:
//	bool init();
	yarp::dev::PolyDriver *motordriver;
	yarp::dev::IPositionControl *pos;
	yarp::dev::IEncoders *enc;

	//Joint IDs
	const static int headX = 2;
	const static int headY = 0;

	//Movement ranges
//	const static double xMin = -35.0;
//	const static double xMax =  35.0;
//	const static double yMin = -35.0;
//	const static double yMax =  18.0;



	//	The above ranges cause it to take a long time to learn, with the range below, it should learn much faster
//	const static double xMin = -20.0;
//	const static double xMax =  20.0;
//	const static double yMin = -30.0;
//	const static double yMax =  18.0;
};

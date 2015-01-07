/*
 * graspController.h
 *
 *  Created on: 1 Jun 2012
 *      Author: icub
 */

#ifndef GRASP_CONTROLLER
	#define GRASP_CONTROLLER

#include <iostream>
#include <stdlib.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>

#include "armController.h"



const short SAMPLES = 100;
const short SENSORS = 60;
const float TOUCH_THRESHOLD=3.0;	//1.5

class graspController //: public armController
{
	public:

		graspController(std::string robot, armController* arm);
		bool initGrasp(std::string robot);
		bool move(const double* position,bool block=true, const bool rightArm=true);
		bool handsStationary();
		bool grasp(bool rightArm=true);

		bool release(bool rightArm=true);
		bool openThumb(bool rightArm);
		bool closeThumb(bool rightArm);

		bool isHolding(){return grasping;}


	private:
		yarp::os::BufferedPort < yarp::os::Bottle > portRight;
		yarp::os::BufferedPort < yarp::os::Bottle > portLeft;
//		double currentRight[SENSORS];
//		double currentLeft[SENSORS];
//		int size;
//		double threshold;
		yarp::os::Bottle* data;

		bool grasping;
//		bool handEmpty;

		float avgRight[SENSORS];
		float avgLeft[SENSORS];

		float sdRight[SENSORS];
		float sdLeft[SENSORS];

		float maxDiffRight[SENSORS];
		float maxDiffLeft[SENSORS];

		armController* ac;
//		yarp::dev::IVelocityControl *velRight;
//		yarp::dev::IVelocityControl *velLeft;
//
//		static const double speedClose = 30;
//		static const double speedOpen = -26;
};

#endif

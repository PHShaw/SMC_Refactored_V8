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

		graspController(std::string robot, armController* arm, bool skin=true);

		bool initGrasp(std::string robot);
		void closePorts();
		bool move(const double* position,bool block=true, const bool rightArm=true);
		bool handsStationary();
		bool grasp(bool rightArm=true);

		bool release(bool rightArm=true, bool block=true);
		bool openThumb(bool rightArm);
		bool closeThumb(bool rightArm);

		bool fist(bool rightArm, bool block=true);

		bool isHolding(){return grasping;}

		bool isHandOpen() const
		{
			return handOpen;
		}

	private:
		yarp::os::BufferedPort < yarp::os::Bottle > portRight;
		yarp::os::BufferedPort < yarp::os::Bottle > portLeft;
//		double currentRight[SENSORS];
//		double currentLeft[SENSORS];
//		int size;
//		double threshold;
		yarp::os::Bottle* data;

		bool grasping;
		bool skin;
//		bool handEmpty;

		float avgRight[SENSORS];
		float avgLeft[SENSORS];

		float sdRight[SENSORS];
		float sdLeft[SENSORS];

		float maxDiffRight[SENSORS];
		float maxDiffLeft[SENSORS];

		bool handOpen;	//true is open, false is closed (fisted/grasping)

		armController* ac;
//		yarp::dev::IVelocityControl *velRight;
//		yarp::dev::IVelocityControl *velLeft;
//
//		static const double speedClose = 30;
//		static const double speedOpen = -26;
};

#endif

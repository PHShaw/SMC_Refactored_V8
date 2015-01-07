/*
 * vor2.cpp
 *
 *  Created on: 29th March 2011
 *      Author: icub
 */

#ifndef TRACKER_PHS
	#define TRACKER_PHS


#include <boost/thread/thread.hpp>
//#include <boost/thread/xtime.hpp>
#include <fstream>
#include <iostream>
#include <string.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include "Target.h"

//using namespace std;
//using namespace yarp::os;
//using namespace yarp::dev;
//using namespace yarp::sig;
//using namespace boost;

/**
 * NOTE: The degrees on the x axis go in the opposite direction
 * Head range is -50 to +50, left to right looking at the icub, while
 * Eye range is +30 to -30.
 *
 * In the y dimensions, the ranges are:
 * Head range: +20 to -35, up to down,
 * Eye range: +18 to -35
 *
 * The relation between the eye and head movements is pretty much linear
 * and almost 1 to 1, but not quite.  This shows up more in the vertical,
 * and particularly near the motor limits
 *
 *
 * While the centre of the x axis is roughly the same for the eyes
 * and head, the y axis may be more offset.  If this is significant,
 * it may be necessary to include some compensation for the offset.
 */

class tracker{
public:


	tracker()
	{
		colour = YELLOW;
	}

	tracker(std::string pCol)
	{
		colour = pCol;
	}
	tracker(std::string pCol, EyeHeadSaccading* ehc)
	{
		colour = pCol;
		initTrack(ehc);
	}

	bool initTrack(EyeHeadSaccading* ehc)
	{
		ehCont = ehc;
		targ = ehCont->getTarget();
		motordriver = ehCont->getMotorDriver();

		bool ok;
		ok = motordriver->view(pos);
		ok &= motordriver->view(enc);

		if (!ok)
		{
			std::cout << "Failed to obtain position or encoder interface" << std::endl;
		}

		counter = 0;
		tracklogfile.open("../../data/tracklog.txt");
		tracklogfile << "x y" << std::endl;



		return ok;
	}

	bool initTrack(yarp::dev::PolyDriver *pmotor, Target* target, EyeHeadSaccading* ehc)
	{
		targ = target;
		ehCont = ehc;
		motordriver = pmotor;
		bool ok;
		ok = motordriver->view(pos);
		ok &= motordriver->view(enc);

		if (!ok)
		{
			std::cout << "Failed to obtain position or encoder interface" << std::endl;
		}

		counter = 0;
		tracklogfile.open("../../data/tracklog.txt");
		tracklogfile << "x y" << std::endl;



		return ok;
	}

	bool stationary()
	{
		bool motionCompleted = false;
		bool x=false, y=false;
		bool success;
		success = pos->checkMotionDone(eyeX, &x);
		success &= pos->checkMotionDone(eyeY, &y);

		if(!success)
		{
			std::cout << "Error whilst attempting to check if eye motion done" << std::endl;
		}
		motionCompleted = x && y;
		if(motionCompleted)
		{
//			cout << "eye motion done" << endl;
			return true;
		}
		else
		{
	//		cout << "still moving" << endl;
			return false;
		}
	}

	void track(){

		tracklogfile << "target tracking " << counter << std::endl;
		counter ++;

//		std::string colour = YELLOW;
		double targX, targY;

		while(true)
		{
			targ->getTarget(&targX, &targY, colour);
			if(!targ->targetCentred() && targ->targetVisible())
			{
				ehCont->fixate(targX, targY, colour, true);
			}
			else if(!targ->targetVisible()){
				ehCont->centreEyesInHead();
			}
			else{
				ehCont->centreEyesInHead();
				ehCont->autoCenter(colour);
			}

			while(!stationary())
					yarp::os::Time::delay(0.1);


//			double testX,testY;
//			enc->getEncoder(eyeY,&testY);
//			enc->getEncoder(eyeX,&testX);
//			if(testX>=25 || testX <=-25 || testY>=15 || testY<=-30)
//			{
//				cout <<"recentering eye!" << endl;
//				ehCont->centreEyesInHeadOnTarget(colour);
//			}


			boost::this_thread::interruption_point();	//throws boost::thread_interrupted
												//if ..interrupt_enabled() and
												// ..interrupt_requested() both return true

	//		Time::delay(0.1);

		}

	}



	void closeLog()
	{
		tracklogfile << "**************END**************" << endl;
		tracklogfile.close();
	}

private:
	yarp::dev::PolyDriver *motordriver;
	yarp::dev::IPositionControl *pos;
	yarp::dev::IEncoders *enc;

	Target* targ;
	EyeHeadSaccading* ehCont;

	std::string colour;

	std::ofstream tracklogfile;	//Can't write to a log file here due to the void ()() operator


	//Joint IDs
	const static int headX = 2;
	const static int headY = 0;
	const static int eyeX = 4;
	const static int eyeY = 3;

	const static int eyeXmax = 28;
	const static int eyeXmin = -28;
	const static int eyeYmax = 18;
	const static int eyeYmin = -35;

	int counter;

};


#endif

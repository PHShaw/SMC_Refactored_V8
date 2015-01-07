/*
 * torsoCompensator.h
 *
 *  Created on: 29th March 2011
 *      Author: icub
 */

#ifndef TORSO_COMPENSATOR_PHS
	#define TORSO_COMPENSATOR_PHS


#include <boost/thread/thread.hpp>
//#include <boost/thread/xtime.hpp>
#include <fstream>
#include <iostream>
#include <string.h>


#include "CalcGazeChange.h"

#include <yarp/os/all.h>
#include <yarp/dev/all.h>


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

class torsoCompensator{
public:


	bool initTorsoCompensator(yarp::dev::PolyDriver *pTorsoMotor, yarp::dev::PolyDriver *pHeadMotor)
	{
		torsoMotordriver = pTorsoMotor;
		headMotordriver = pHeadMotor;
		bool ok;
		ok = torsoMotordriver->view(torsoPos);
		ok &= torsoMotordriver->view(torsoEnc);

		ok &= headMotordriver->view(headPos);
		ok &= headMotordriver->view(headEnc);
		if (!ok)
		{
			std::cout << "Failed to obtain position or encoder interface" << std::endl;
		}

		counter = 0;
		torsoComplogfile.open("../data/torsoComplog.txt");
		torsoComplogfile << "yaw pitch overX overY" << std::endl;


		return ok;
	}

	bool stationary()		// test to see if head is stationary
	{
		bool h=false, t=false;
		bool success;
		success = headPos->checkMotionDone(&h);
		success &= torsoPos->checkMotionDone(&t);

		return h && t && success;
	}

	void track(){

		double* torsoMotorConf1 = new double[3];
		double* headMotorConf1 = new double[6];
		torsoEnc->getEncoders(torsoMotorConf1);
		headEnc->getEncoders(headMotorConf1);

//		printf("Current: [%.1f,%.1f,%.1f][%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,]\n",torsoMotorConf1[0],torsoMotorConf1[1],torsoMotorConf1[2],
//				headMotorConf1[0],headMotorConf1[1],headMotorConf1[2],headMotorConf1[3],headMotorConf1[4],headMotorConf1[5]);

		double* torsoMotorConf2 = new double[3];
		double* headMotorConf2 = new double[6];


		torsoComplogfile << "VOR compensation " << counter << std::endl;
		counter ++;

		while(true)
		{

//			while(!stationary())
					yarp::os::Time::delay(0.2);


			torsoEnc->getEncoders(torsoMotorConf2);
			headEnc->getEncoders(headMotorConf2);
//			printf("New:     [%.1f,%.1f,%.1f][%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,]\n",torsoMotorConf1[0],torsoMotorConf1[1],torsoMotorConf1[2],
//				headMotorConf1[0],headMotorConf1[1],headMotorConf1[2],headMotorConf1[3],headMotorConf1[4],headMotorConf1[5]);

			//Look to see if the torso has moved.
			if( torsoMotorConf1[torsoYaw]  !=torsoMotorConf2[torsoYaw] ||
				torsoMotorConf1[torsoPitch]!=torsoMotorConf2[torsoPitch])
			{
				//give compensated actual eye values as eye to head not 1 to 1


				//Torso Yaw to Head Yaw ~ 1:1
				//Torso Pitch to Head Pitch ~ 1:1 until head = +18, after which, eye takes over at f(x)=1.14x-17.4


				//if using the kinematics, use torso:head compensation estimates as above,
				//then calculate remainder as eye movement to complete compensation.
				//Remember the kinematics may return actual motor values.

				double yawDiff = torsoMotorConf2[torsoYaw] - torsoMotorConf1[torsoYaw];
				double pitchDiff = torsoMotorConf2[torsoPitch]-torsoMotorConf1[torsoPitch];



				if(yawDiff+headMotorConf2[headX] > headXmax)
					headMotorConf2[headX] = headXmax;
				else if(yawDiff+headMotorConf2[headX] < headXmin)
					headMotorConf2[headX] = headXmin;
				else
					headMotorConf2[headX] += yawDiff;

				if(pitchDiff+headMotorConf2[headY] > headYmax)
					headMotorConf2[headY] = headYmax;
				else if(pitchDiff+headMotorConf2[headY] < headYmin)
					headMotorConf2[headY] = headYmin;
				else
					headMotorConf2[headY] += pitchDiff;


				double tilt, version, vergence;
				//Returns relative pan and tilt values
				CalculateGazeChange(torsoMotorConf1,headMotorConf1,torsoMotorConf2,headMotorConf2, &tilt, &version, &vergence);
				double x = headMotorConf2[eyeX]+version;	//Actual motor values
				double y = headMotorConf2[eyeY]+tilt;



				//Include the previous overflow into the difference
				if(xOverflow != 0)
				{
					x += xOverflow;
					xOverflow = 0;
				}
				if(yOverflow != 0)
				{
					y += yOverflow;
					yOverflow = 0;
				}

				//Check for new overflow
				if(x>eyeXmax)
				{
					xOverflow = x - eyeXmax;
					x = eyeXmax;
				}
				else if(x<eyeXmin)
				{
					xOverflow = x - eyeXmin;
					x = eyeXmin;
				}

				if(y>eyeYmax)
				{
					yOverflow = y - eyeYmax;
					y = eyeYmax;
				}
				else if(y<eyeYmin)
				{
					yOverflow = y - eyeYmin;
					y = eyeYmin;
				}

				if(xOverflow || yOverflow)
					printf("Torso Overflow (%.2f, %.2f)\n", xOverflow, yOverflow);


				std::cout << "(" << version << "," << tilt << "," << vergence <<")" << std::endl;
				headPos->positionMove(headY, headMotorConf2[headY]);
				headPos->positionMove(headX, headMotorConf2[headX]);
				headPos->positionMove(eyeY, y);
				headPos->positionMove(eyeX, x);



//				torsoComplogfile << x << " " << y << " " << xOverflow << " " << yOverflow << endl;



				for(int i=0; i<3; i++)
				{
					torsoMotorConf1[i] = torsoMotorConf2[i];
				}
				for(int i=0; i<6; i++)
				{
					headMotorConf1[i] = headMotorConf2[i];
				}
			}


			boost::this_thread::interruption_point();	//throws boost::thread_interrupted
													//if ..interrupt_enabled() and
													// ..interrupt_requested() both return true


		}

	}


	bool getOverflow(double* xOver, double* yOver)
	{
		*xOver = xOverflow;
		*yOver = yOverflow;
		return abs(xOverflow) + abs(yOverflow);
	}

	void resetOverflow()
	{
		xOverflow = 0;
		yOverflow = 0;
	}

	void addOverflow(double xOver, double yOver)
	{
		xOverflow += xOver;
		yOverflow += yOver;
	}


	void closeLog()
	{
		torsoComplogfile << "**************END**************" << std::endl;
		torsoComplogfile.close();
	}

private:
	yarp::dev::PolyDriver *torsoMotordriver;
	yarp::dev::PolyDriver *headMotordriver;
	yarp::dev::IPositionControl *torsoPos;
	yarp::dev::IEncoders *torsoEnc;
	yarp::dev::IPositionControl *headPos;
	yarp::dev::IEncoders *headEnc;

	ofstream torsoComplogfile;	//Can't write to a log file here due to the void ()() operator

	double currentTorsoX;
	double currentTorsoY;

	double xOverflow;	//The amount by which the head has moved beyond
	double yOverflow;	// the range of the eye movement.  This should
						// then be included as soon as possible when the
						// head moves again to get the eye back on target.

	//Joint IDs
	const static int torsoPitch = 2;
	const static int torsoYaw = 0;
	const static int torsoRoll = 1;

	const static int headX = 2;
	const static int headY = 0;
	const static int eyeX = 4;
	const static int eyeY = 3;

	const static int headXmax = 35;
	const static int headXmin = -35;
	const static int headYmax = 18;
	const static int headYmin = -35;

	const static int eyeXmax = 30;
	const static int eyeXmin = -30;
	const static int eyeYmax = 18;
	const static int eyeYmin = -35;

	int counter;

};


#endif

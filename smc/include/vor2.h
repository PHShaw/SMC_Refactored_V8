/*
 * vor2.cpp
 *
 *  Created on: 29th March 2011
 *      Author: icub
 */

#ifndef VOR2_PHS
	#define VOR2_PHS


#include <boost/thread/thread.hpp>
//#include <boost/thread/xtime.hpp>
#include <fstream>
#include <iostream>
#include <string.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

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

class vor{
public:

//	~vor()
//	{
//		motordriver->close();
//	}

//	vor(){}
//	vor(const vor&) {}		//Added to satisfy ofstream

	bool initVor(yarp::dev::PolyDriver *pmotor)
	{
		motordriver = pmotor;
		bool ok;
		ok = motordriver->view(pos);
		ok &= motordriver->view(enc);

		if (!ok)
		{
			std::cout << "Failed to obtain position or encoder interface" << std::endl;
		}

		counter = 0;
		vorlogfile.open("../../data/vorlog.txt");
//		vorlogfile << "x y overX overY" << std::endl;
		vorlogfile << "headX headY eyeX eyeY timeStamp" << std::endl;

//		enc->getEncoder(headX,&currentHeadX);
//		enc->getEncoder(headY,&currentHeadY);
//
//		xOverflow = 0;
//		yOverflow = 0;

		return ok;
	}

	bool stationary()
	{
//		double testX1, testY1, testX2, testY2;
//		enc->getEncoder(eyeY,&testY1);
//		enc->getEncoder(eyeX,&testX1);
//		yarp::os::Time::delay(0.1);
//		enc->getEncoder(eyeY,&testY2);
//		enc->getEncoder(eyeX,&testX2);
//	//	cout << testX1 << " " << testX2 << endl;
//		if((int)testX1!=(int)testX2 && (int)testY1!=(int)testY2)
//			return false;
//		else
//			return true;

		bool success,x=false,y=false;
		success = pos->checkMotionDone(eyeX, &x);
		success &= pos->checkMotionDone(eyeY, &y);
		return x&&y;
	}

//	void operator()() { track();}
	void track(){
//		xOverflow = 0;
//		yOverflow = 0;

		enc->getEncoder(headX,&currentHeadX);
		enc->getEncoder(headY,&currentHeadY);

		vorlogfile << "VOR compensation " << counter << std::endl;
		counter ++;

		while(true)
		{

			while(!stationary())
					yarp::os::Time::delay(0.01);

			double newHeadX, newHeadY;
			double currentEyeX, currentEyeY;
			enc->getEncoder(headX, &newHeadX);
			enc->getEncoder(headY, &newHeadY);

			enc->getEncoder(eyeX, &currentEyeX);
			enc->getEncoder(eyeY, &currentEyeY);
			time_t current = time(NULL);
			//vorlogfile << "headX headY eyeX eyeY timeStamp" << std::endl;
			vorlogfile << newHeadX << " " << newHeadY << " " << currentEyeX << " " << currentEyeY << " " << current << std::endl;

			//Look to see if the head has moved.
			if(currentHeadX!=newHeadX || currentHeadY!=newHeadY)
			{
				//give compensated actual eye values as eye to head not 1 to 1
				double compNewHeadtoEyeX, compNewHeadtoEyeY, compCurrentHeadtoEyeX, compCurrentHeadtoEyeY;

//				compNewHeadtoEyeX = 1.2*newHeadX + 0.34;
//				compCurrentHeadtoEyeX = 1.2*currentHeadX +0.34;
				compNewHeadtoEyeX = 1.02*newHeadX + 0.034;
				compCurrentHeadtoEyeX = 1.02*currentHeadX +0.034;

				compNewHeadtoEyeY = -1.2*newHeadY +0.84;
				compCurrentHeadtoEyeY = -1.2*currentHeadY +0.84;

//				double xDiff = newHeadX - currentHeadX;
//				double yDiff = newHeadY - currentHeadY;

				double xDiff = compNewHeadtoEyeX - compCurrentHeadtoEyeX;
				double yDiff = compNewHeadtoEyeY - compCurrentHeadtoEyeY;






				//Convert eye ranges to relative available movement
				double Xmax, Xmin, Ymax, Ymin;
				Xmax = eyeXmax-currentEyeX;
				Xmin = eyeXmin-currentEyeX;
				Ymax = eyeYmax-currentEyeY;
				Ymin = eyeYmin-currentEyeY;

//				cout << Xmax << " " << Xmin << endl;

				//Include the previous overflow into the difference
				if(xOverflow != 0)
				{
					xDiff += xOverflow;
					xOverflow = 0;
				}
				if(yOverflow != 0)
				{
					yDiff += yOverflow;
					yOverflow = 0;
				}

				//Check for new overflow
				if(xDiff>Xmax)
				{
					xOverflow = xDiff - Xmax;
					xDiff = Xmax;
					std::cout << "xOverflow: " << xOverflow << std::endl;
				}
				else if(xDiff<Xmin)
				{
					xOverflow = xDiff - Xmin;
					xDiff = Xmin;
					std::cout << "xOverflow: " << xOverflow << std::endl;
				}

				if(yDiff>Ymax)
				{
					yOverflow = yDiff - Ymax;
					yDiff = Ymax;
					std::cout << "yOverflow: " << yOverflow << std::endl;
				}
				else if(yDiff<Ymin)
				{
					yOverflow = yDiff - Ymin;
					yDiff = Ymin;
					std::cout << "yOverflow: " << yOverflow << std::endl;
				}

//				cout << "xOverflow: " << xOverflow << endl;
//				cout << "yOverflow: " << yOverflow << endl;

				//Convert relative eye movements back to actual motor
				//values to make the movements
				double x,y;
				x = xDiff + currentEyeX;
				y = yDiff + currentEyeY;
				std::cout << "(" << x << "," << y << ")" << std::endl;
				pos->positionMove(eyeY, y);
				pos->positionMove(eyeX, x);



				//vorlogfile << x << " " << y << " " << xOverflow << " " << yOverflow << std::endl;




				currentHeadX = newHeadX;
				currentHeadY = newHeadY;
			}


			/*
			boost::xtime xt;
			boost::xtime_get(&xt, boost::TIME_UTC);
			xt.sec+=1;		//is 1 second really the shortest waiting
							//time I can get without using a lock & wait
			boost::thread::sleep(xt);
			*/

			boost::this_thread::interruption_point();	//throws boost::thread_interrupted
												//if ..interrupt_enabled() and
												// ..interrupt_requested() both return true

	//		Time::delay(0.1);

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
		vorlogfile << "**************END**************" << std::endl;
		vorlogfile.close();
	}

private:
	yarp::dev::PolyDriver *motordriver;
	yarp::dev::IPositionControl *pos;
	yarp::dev::IEncoders *enc;

	std::ofstream vorlogfile;	//Can't write to a log file here due to the void ()() operator

	double currentHeadX;
	double currentHeadY;

	double xOverflow;	//The amount by which the head has moved beyond
	double yOverflow;	// the range of the eye movement.  This should
						// then be included as soon as possible when the
						// head moves again to get the eye back on target.

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

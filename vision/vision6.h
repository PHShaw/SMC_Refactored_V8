/*
 * vision6.h
 *
 *  Created on: 7 May 2015
 *      Author: icub
 */

#ifndef VISION6_H_
#define VISION6_H_

#include <yarp/os/all.h>	//bottle, bufferedPort
#include <yarp/sig/all.h>	//imageOf, PixelPgb

#include <signal.h>		//used in the save and quit functions
#include <algorithm>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


class Vision : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
	public:
		Vision();
		void init(std::string pcam);

		void findTarget(yarp::sig::ImageOf<yarp::sig::PixelBgr>* imgin, yarp::sig::ImageOf<yarp::sig::PixelBgr>* imgout);
		yarp::sig::ImageOf<yarp::sig::PixelBgr> adjustFOV(int percent, yarp::sig::ImageOf<yarp::sig::PixelBgr>* imgin);
		yarp::sig::ImageOf<yarp::sig::PixelBgr> adjustAccuity(int percent, yarp::sig::ImageOf<yarp::sig::PixelBgr>* imgin);
		bool fovea(int x, int y, double* dist);

		yarp::sig::ImageOf<yarp::sig::PixelBgr> colourSwitch(yarp::sig::ImageOf<yarp::sig::PixelBgr>* imgin);



		virtual void onRead(yarp::os::Bottle& b);
		void setAcuity(int value);
		void setFov(int value);

		void run();

		void closePorts();

	private:

		 yarp::os::BufferedPort <  yarp::os::Bottle > porttargets;
		 yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelBgr> > camPortin;
		 yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelBgr> > camPortout;

		 yarp::os::BufferedPort < yarp::os::Bottle> visionParamsIn;

		 std::string cam;

		int width, height;
		int fov, acuity;

		bool checkRed;
		bool checkYellow;
		bool checkBlue;
		bool checkGreen;
		bool checkWhite;
		bool checkGrey;
		IplImage *cvImage;	//if this ceases to exist, then wrapped version segfaults

};



#endif /* VISION6_H_ */

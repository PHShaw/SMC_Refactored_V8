/*
 * monitor.h
 *
 *  Created on: 15 Oct 2013
 *      Author: icub
 */

#ifndef MONITOR_H_
#define MONITOR_H_

#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <string.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>


class monitor{
public:
	bool initMonitor(yarp::dev::PolyDriver *headMotor, yarp::dev::PolyDriver *torsoMotor,
					 yarp::dev::PolyDriver *lArmMotor, yarp::dev::PolyDriver *rArmMotor)
	{
		bool ok;
		ok = headMotor->view(headEnc);
		ok &= torsoMotor->view(torsoEnc);
		ok &= lArmMotor->view(leftArmEnc);
		ok &= rArmMotor->view(rightArmEnc);

		monitorfile.open("../../data/monitorlog.txt");

		return ok;
	}
	void closeLog()
	{
		monitorfile.close();
	}


	void record(){
		double hx,hy,ex,ey,ev,tx,ty;	// pan/yaw, pitch/tilt, vergence
		double la1,la2,la3,la4;	//left arm, first 4 joints
		double ra1,ra2,ra3,ra4;	//right arm, first 4 joints
		bool success;
		time_t current;
		while(true)
		{
			yarp::os::Time::delay(0.005);

			success = headEnc->getEncoder(headX, &hx);
			success &= headEnc->getEncoder(headY, &hy);
			if(success)
			{
				current = time(NULL);
				monitorfile << "head " << hx << " " << hy << "   " << current << std::endl;
			}

			success = headEnc->getEncoder(eyeX, &ex);
			success &= headEnc->getEncoder(eyeY, &ey);
			success &= headEnc->getEncoder(eyeV, &ev);
			if(success)
			{
				current = time(NULL);
				monitorfile << "eye " << ex << " " << ey << " " << ev << "  " << current << std::endl;
			}

			success = torsoEnc->getEncoder(torX, &tx);
			success &= torsoEnc->getEncoder(torY, &ty);
			if(success)
			{
				current = time(NULL);
				monitorfile << "torso " << tx << " " << ty << "   " << current << std::endl;
			}

			success = leftArmEnc->getEncoder(0, &la1);
			success &= leftArmEnc->getEncoder(1, &la2);
			success &= leftArmEnc->getEncoder(2, &la3);
			success &= leftArmEnc->getEncoder(3, &la4);
			if(success)
			{
				current = time(NULL);
				monitorfile << "lArm " << la1 << " " << la2 << " " << la3 << " " << la4 << " " << current << std::endl;
			}

			success = rightArmEnc->getEncoder(0, &ra1);
			success &= rightArmEnc->getEncoder(1, &ra2);
			success &= rightArmEnc->getEncoder(2, &ra3);
			success &= rightArmEnc->getEncoder(3, &ra4);
			if(success)
			{
				current = time(NULL);
				monitorfile << "rArm " << ra1 << " " << ra2 << " " << ra3 << " " << ra4 << " " << current << std::endl;
			}

			boost::this_thread::interruption_point();
		}
	}

private:
	yarp::dev::IEncoders *headEnc;
	yarp::dev::IEncoders *leftArmEnc;
	yarp::dev::IEncoders *rightArmEnc;
	yarp::dev::IEncoders *torsoEnc;

	std::ofstream monitorfile;

	//Joint IDs
	const static int headX = 2;
	const static int headY = 0;
	const static int eyeX = 4;
	const static int eyeY = 3;
	const static int eyeV = 5;
	const static int torX = 0;
	const static int torY = 2;
};


#endif /* MONITOR_H_ */

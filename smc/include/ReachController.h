/*
 * ReachController.h
 *
 * This class interacts with the separate reach system to direct reaches towards the target
 *
 *  Created on: 9 May 2015
 *      Author: icub
 */

#ifndef REACHCONTROLLER_H_
#define REACHCONTROLLER_H_
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/PortReaderBuffer.h>	//contains typed reader callback class
#include <yarp/os/Network.h>

#include <iostream>
#include <string>
namespace smc
{


	const std::string msg_COMPLETE = "complete";
	const std::string msg_COMPLETED = "completed";
	const std::string msg_WAITING = "waiting";
	const std::string msg_HOME = "home";
	const std::string msg_STOPPED = "stopped";
	const std::string msg_REACHING = "reaching";
	const std::string msg_UNREACHABLE = "unreachable";

	enum Status {COMPLETE, HOME, WAITING, STOPPED, REACHING, UNREACHABLE, UNKNOWN};


	class ReachController : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
	{
		public:
			ReachController();
			virtual ~ReachController();

			void openPorts();
			void closePorts();

			virtual void onRead(yarp::os::Bottle& b);

			void sendArmTarget(double x, double y, double z, bool right_arm);
			bool reachToPoint(double wx, double wy, double wz, double* dist);
		//	float getReachFeedback();
			void command(std::string cmd);
			void command(std::string cmd, bool arm);

			Status getCurrentStatus(){return status;}
			float getDistance(){return dist;}
			bool isReachingOkay();

		private:
			yarp::os::BufferedPort<yarp::os::Bottle> portReachCommands;
			yarp::os::BufferedPort<yarp::os::Bottle> portReachFeedback;
			yarp::os::Bottle* reachResponse;

			float dist;
			int ratio;	//quality of current reach, >2 good, <2 bad;
			Status status;


	};

} /* namespace std */
#endif /* REACHCONTROLLER_H_ */

/*
 * ReachController.cpp
 *
 *  Created on: 9 May 2015
 *      Author: icub
 */

#include "ReachController.h"
using namespace std;

namespace smc
{

ReachController::ReachController()
{
	status = HOME;
	dist = 0;
	x = 0;
	y = 0;
	z = 0;
	ratio= 1;
	arms = 'b';
}

ReachController::~ReachController()
{
	closePorts();
}


void ReachController::openPorts()
{
	yarp::os::Network yarp;

	portReachCommands.open("/smc/reaching/out");
	portReachFeedback.open("/smc/reaching/in");
	portReachFeedback.useCallback(*this);
}

void ReachController::closePorts()
{
	portReachCommands.close();
	portReachFeedback.close();
}


void ReachController::onRead(yarp::os::Bottle& b) {
	 std::cout<<"[BOTTLE] Reach Feedback received: '"<<b.toString()<<"' size: "<<b.size()<<std::endl;
	 //[status dist x y z]
	 string message = b.get(0).asString().c_str();
	 if(message == msg_COMPLETE || message == msg_COMPLETED)
	 {
		 status = COMPLETE;
		 dist = b.get(1).asDouble();
		 x = b.get(2).asDouble();
		 y = b.get(3).asDouble();
		 z = b.get(4).asDouble();
	 }
	 else if(message == msg_HOME)
	 {
		 status = HOME;
		 string which = b.get(1).asString().c_str();
		 arms = which[0];
		 dist = 0;
		 x = b.get(2).asDouble();
		 y = b.get(3).asDouble();
		 z = b.get(4).asDouble();
	 }
	 else if(message == msg_WAITING)
	 {
		 status = WAITING;
	 }
	 else if(message == msg_REACHING)
	 {
		 status = REACHING;
		 dist = b.get(1).asDouble();
		 x = b.get(2).asDouble();
		 y = b.get(3).asDouble();
		 z = b.get(4).asDouble();
		 ratio = b.get(5).asInt();	//Quality of reach, 1 is good, 0 is bad.
	 }
	 else if(message == msg_STOPPED)
	 {
		 status = STOPPED;
	 }
	 else if(message == msg_UNREACHABLE)
	 {
		 status = UNREACHABLE;
	 }
	 else
	 {
		 status = UNKNOWN;
	 }
}

/**
 * Checks the ratio returned during reaching.  If the ratio (good/bad moves) is 0, then bad.
 */
bool ReachController::isReachingOkay()
{
	return ratio>=1;
}


/**
 * coordinates should be given in meters, e.g. -0.28 0.13 0.13
 */
void ReachController::sendArmTarget(double x, double y, double z, bool right_arm)
{
	if(right_arm)
		arms = 'r';
	else
		arms = 'l';

	yarp::os::Bottle& b = portReachCommands.prepare();
	b.clear();
	b.addString("target");
	if(right_arm)
		b.addString("right_arm");
	else
		b.addString("left_arm");
	b.addDouble(x);	// /1000
	b.addDouble(y);
	b.addDouble(z);
	//can add "horizontal" or "vertical" at end of this message, but defaults to "horizontal"
	b.addString("horizontal");
	portReachCommands.write();
}


void ReachController::command(string cmd)
{
	arms = 'b';
	//Send command to reaching
	yarp::os::Bottle& b = portReachCommands.prepare();
	b.clear();
	b.addString(cmd.c_str());
	portReachCommands.write(true);
}

void ReachController::command(string cmd, bool arm)
{
	if(arm)
		arms = 'r';
	else
		arms = 'l';
	//Send command to reaching
	yarp::os::Bottle& b = portReachCommands.prepare();
	b.clear();
	b.addString(cmd.c_str());
	if(arm)
		b.addString("right_arm");
	else
		b.addString("left_arm");
	portReachCommands.write(true);
}

/**
 * Blocks until a message has been received
 *
float getReachFeedback()
{
	string message;
	yarp::os::Bottle* reachResponse;
	float dist;
	   do{
			yarp::os::Time::delay(0.3);
			reachResponse=portReachFeedback.read(true);
		 std::cout<<"[BOTTLE] received: '"<<reachResponse->toString()<<"' size: "<<reachResponse->size()<<std::endl;
			message = reachResponse->get(0).asString().c_str();
//			cout << "received: " << message << endl;
		}while(message != "completed" && message != "complete");
	   //Also receive "wait" "home"
		dist = reachResponse->get(1).asDouble();
		cout << "Reach distance: " << dist << endl;

	//[stage] [dist] ?

	return dist;
}
*/

	std::string ReachController::statusToString(Status status)
	{
		switch(status)
		{
			case COMPLETE:
				return "complete";
				break;
			case HOME:
				return "home";
				break;
			case WAITING:
				return "waiting";
				break;
			case STOPPED:
				return "stopped";
				break;
			case REACHING:
				return "reaching";
				break;
			case UNREACHABLE:
				return "unreachable";
				break;
			case UNKNOWN:
			default:
				return "unknown";
				break;
		}
		return "unknown";
	}


} /* namespace smc */



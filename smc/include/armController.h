/*
 * Arm controller controls the yarp interface for both left and right arms.
 * When initialising, can specify whole arm or just the hand.  Just the hand
 * is used when the reaching is controlled by the separate reaching software.
 *
 *  Created on: 8 Aug 2011
 *      Author: icub
 */

#ifndef ARM_CONTROLLER
	#define ARM_CONTROLLER

	#define LEFT_ARM
	#define RIGHT_ARM	//used to indicate functional arms

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>


#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include "utilities.h"
//#include "ReachSpace.h"
//#include "ReachSpaceJoyStick.h"		//original board reach configs
#include "dtat2.h"	//James' initial reach configs - fixed elbow, unknown palm


//									  0	   1	2	 3	  4    5    6   7   8   9   10  11   12  13   14  15
const double absMinJointRanges[] = {-95,  10, -35,  15, -80, -65, -15,  0,  5,  5,   0,  0,   0,  0,   0,  0};
const double absMaxJointRanges[] = {  5, 140,  75, 100,  50,   0,  25, 60, 90, 90, 180, 90, 175, 90, 175, 200};


const double ARM_REST[] =	 	{  5, 10,  0, 15,  0,-30,  0, 55,  0, 66, 32,  5,  5,  5,  5,  5};
const double ARM_BOARD_REST[] =	{-38, 64, 50, 15, 20,-30,  0, 55,  0, 66, 32,  5,  5,  5,  5,  5};
const double ARM_VELOCITY[] = 	{ 12, 12, 12, 10, 10, 10, 10, 20, 40, 50, 60, 60, 70, 70, 80, 100};

class graspController;


class armController{
public:
	armController(bool grippyOnly, bool board=false);
	~armController();

	bool armsStationary();
	void move(const double* position, bool block=true, bool rightArm=true, bool moveHand=false);
	void move(int pos, bool block=true, bool rightArm=false, bool moveHand=false);


	/**
	 * NOT ADVISABLE TO DISABLE SAFEMODE
	 */
	void disableSafeMode(){safeMode=false;}
	void enableSafeMode(){safeMode=true;}

	bool elbowCheck(const double* position, bool rightArm=true);

	void stopMovement();
	void generateRandomPosition(double* reach, const int noJoints=4);
	void generateSmallRelativeMovement(const double* currentPos, double* reach, const int noJoints=4, int maxRange=5);
	bool babble(const int arm=RIGHT);		//Returns a boolean indicating if it was the right or left arm that moved... right arm = true.
	bool smallBabble(const bool arm=RIGHT, int maxRange=10);

	void armsToRest();		//Moves both arms to rest
	void armRest(const bool rightArm=true);	//move selected arm to rest.

	void pressButton(const bool rightArm=true);
	void smallPush(const bool rightArm=true);

	bool getCurrentPosition(double* position, const bool rightArm=true);
	bool isAtRest(const bool rightArm=true);
	bool isAtPosition(const double* compPos, const bool rightArm=true);
	bool isAtPosition(const double* pos1, const double* pos2);

	void armAction(int action, const double* position=ARM_BOARD_REST, bool rightArm=false);
	void reachAndPressLeft(const double* position);
	void reachAndPressRight(const double* position);
	void reachAndPushLeft(const double* position);
	void reachAndPushRight(const double* position);
	void reachLeft(const double* position);
	void reachRight(const double* position);

	//Currently Pseudo calculated using the flexion and rotation joints.
	//Note, left arm angle should be inverted.
	double getDepth(bool rightArm=true);
	double getAngle(bool rightArm=true);
	bool getStretch(double* j1, double* j3, bool rightArm);

//	bool armsIntersecting();

	yarp::dev::PolyDriver* getLeftMotorDriver(){return leftArmDriver;}
	yarp::dev::PolyDriver* getRightMotorDriver(){return rightArmDriver;}


//	void increaseMovementRange(int percent);	//TODO
     double* getMaxJointRanges();
     double* getMinJointRanges();
    void setMaxJointRanges(double* maxJointRanges);
    void setMinJointRanges(double* minJointRanges);


	static const int LEFT = 0;
	static const int RIGHT = 1;
	static const int BOTH = 2;

	//Available actions
	static const int REACH = 1;
	static const int REACH_AND_PRESS = 2;
	static const int PRESS = 3;
	static const int REST = 4;
	static const int REACH_AND_PRESS_LEFT = 5;
	static const int REACH_AND_PRESS_RIGHT = 6;
	static const int REACH_LEFT = 7;
	static const int REACH_RIGHT = 8;

	static const int SMALL_PUSH = 9;
	static const int REACH_AND_PUSH_LEFT = 10;
	static const int REACH_AND_PUSH_RIGHT = 11;

	friend class graspController;

protected:
	bool initYarp();
//	int randGenerator(const int range);

	yarp::dev::PolyDriver *leftArmDriver;
	yarp::dev::IPositionControl *leftArmPos;
	yarp::dev::IEncoders *leftArmEnc;

	yarp::dev::PolyDriver *rightArmDriver;
	yarp::dev::IPositionControl *rightArmPos;
	yarp::dev::IEncoders *rightArmEnc;


	//Arm ranges:
	double* minJointRanges;
	double* maxJointRanges;


	//Arm joint IDs
	//Rotational joints:
	 static const int shoulderPitch = 0;
	 static const int shoulderYaw = 2;
	 static const int wristProSup = 4; //wrist pronosupination
	 static const int wristYaw = 6;
	//Flexion joints:
	 static const int shoulderRoll = 1;
	 static const int elbow = 3;
	 static const int wristPitch = 5;
	//Hand joint IDs:
	 static const int handAdduction = 7; //adduction/abduction (spreading fingers out)
	 static const int thumbOppose = 8;
	 static const int thumbProximal = 9;
	 static const int thumbDistal = 10;
	 static const int indexProximal = 11;
	 static const int indexDistal = 12;
	 static const int midProximal = 13;
	 static const int midDistal = 14;
	 static const int pinky = 15;

	 bool board;

	 boost::thread_group threads;


	 bool safeMode;
	 bool grippyOnly;
	 bool leftElbowRetracted, rightElbowRetracted;
	 bool armMoving, moveCalled;

	 std::string robot;

	 static const bool checkElbow=true;
};







//Test positions:
//****************
// ARM POSITIONS
//****************
//const double RIGHT_ARM_BUTTON1[]={ -8,28,-5,77,67,-15,0,60,6,66,24,0,0,0,0,0};
//const double RIGHT_ARM_BUTTON2[]={-35,10,19,53,73,-15,0,60,6,66,24,0,0,0,0,0};
//const double LEFT_ARM_BUTTON3[] ={ -8,28,-5,77,67,-15,0,60,6,66,24,0,0,0,0,0};
//
//const double RIGHT_ARM_BOX1[]={-70,23,10,36,64,-15,0,60,6,66,24,0,0,0,0,0};
//const double RIGHT_ARM_BOX2[]={-70,10,10,36,64,-15,0,60,6,66,24,0,0,0,0,0};
//const double LEFT_ARM_BOX3[] ={-70,23,10,36,64,-15,0,60,6,66,24,0,0,0,0,0};

#endif

/*
 * armReach.cpp
 *
 *  Created on: 6 Apr 2011
 *      Author: icub
 */

#include "armController.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


armController::armController(bool handOnly, bool pBoard)
{
	grippyOnly = handOnly;
	board = pBoard;
	initYarp();

//	//Horizontal-ish plane
//	minJointRanges = new double[6];
//	minJointRanges[0]= -80;
//	minJointRanges[1]=  15;
//	minJointRanges[2]=  50;
//	minJointRanges[3]=  15;
//	minJointRanges[4]=   5;
//	minJointRanges[5]= -15;
//
//	maxJointRanges = new double[6];
//	maxJointRanges[0]= -60;
//	maxJointRanges[1]=  45;
//	maxJointRanges[2]=  50;
//	maxJointRanges[3]=  45;
//	maxJointRanges[4]=   5;
//	maxJointRanges[5]= -15;


	if(board)
	{
		minJointRanges = new double[6];
		minJointRanges[0]= -50;
		minJointRanges[1]=  10;
		minJointRanges[2]=  -5;
		minJointRanges[3]=  15;
		minJointRanges[4]=  20;
		minJointRanges[5]= -30;

		maxJointRanges = new double[6];
		maxJointRanges[0]=   0;
		maxJointRanges[1]=  50;
		maxJointRanges[2]=  55;
		maxJointRanges[3]=  90;
		maxJointRanges[4]=  64;
		maxJointRanges[5]= -20;
	}
	else
	{
		//straight arm pointing
		minJointRanges = new double[6];
		minJointRanges[0]= -90;
		minJointRanges[1]=  10;
		minJointRanges[2]=   0;
		minJointRanges[3]=  15;
		minJointRanges[4]=   0;
		minJointRanges[5]= -15;

		maxJointRanges = new double[6];
		maxJointRanges[0]= -55;
		maxJointRanges[1]=  35;
		maxJointRanges[2]=   0;
		maxJointRanges[3]=  15;
		maxJointRanges[4]=   0;
		maxJointRanges[5]= -15;
	}

	safeMode = true;
	board = false;

	leftElbowRetracted = false;
	rightElbowRetracted = false;
	armMoving = false;
	moveCalled = false;

//	if(!grippyOnly)
//		armsToRest();
}


armController::~armController()
{
	leftArmDriver->close();
	rightArmDriver->close();
}


void armController::armsToRest()
{	//first move doesn't block, while second does, so both arms move together.
	//and method doesn't return till both arms at rest.
//	if(board)
//	{
//		move(ARM_BOARD_REST, false, true);	//right arm		//don't block so both move together
//		move(ARM_BOARD_REST, true, false);	//left arm
//	}
//	else
//	{
//		move(ARM_REST, false, true);	//right arm		//don't block so both move together
//		move(ARM_REST, true, false);	//left arm
//	}

#ifdef RIGHT_ARM
	move(HOME_POSE, false, true);	//right arm		//don't block so both move together
#endif
#ifdef LEFT_ARM
	move(HOME_POSE, true, false);	//left arm
#endif
}

void armController::armRest(bool rightArm)
{
		move(HOME_POSE, true, rightArm);
}


void armController::move(int pos, bool block, bool rightArm, bool moveHand)
{


	move(ARM_POSES[pos],block, rightArm, moveHand);


}


bool armController::elbowCheck(const double* position, bool rightArm)
{
	double* current = new double[16];
	if(!getCurrentPosition(current, rightArm))
		return false;

	bool match = true;
	for(int i=0; i<6 ;i++)
	{
//		if(i==elbow)
//			continue;
		if(!((floor(current[i])==(int)position[i]) || (ceil(current[i]) == (int)position[i])))
		{
			match = false;
//			printf("Joint %i doesn't match: %.2f %.2f, ", i, current[i], position[i]);
			break;
		}
	}
	return match;
}


void armController::move(const double* position, bool block, bool rightArm, bool moveHand)
{
//	if(armMoving)
//		stopMovement();
	moveCalled = true;


	if(rightArm)
		cout << "Moving right arm, position requested: [";
	else
		cout << "Moving left arm, position requested: [";
	for(int i=0; i<16; i++)
	{
		cout << position[i];
		if(i!=15)
			cout << ", ";
	}
	cout << "]" << endl;

	double* checked = new double[16];
	//Perform safety check on movement specified
	for(int i=0; i<16; i++)	//Safety ranges only defined for first 6 joints
	{

		if(position[i] < absMinJointRanges[i])
		{
			checked[i] = absMinJointRanges[i];
			printf("Adjusting joint %i from requested %.2f to min %.2f\n", i, position[i], checked[i]);
		}
		else if(position[i] > absMaxJointRanges[i])
		{
			checked[i] = absMaxJointRanges[i];
			printf("Adjusting joint %i from requested %.2f to max %.2f\n", i, position[i], checked[i]);
		}
		else
			checked[i] = position[i];

//		if(i==5)
//		{
//			checked[i] +=20;
//			if(checked[i]>absMaxJointRanges[i])
//				checked[i] = absMaxJointRanges[i];
//		}
	}


	double reserve = checked[elbow];

	if(safeMode)
	{
		cout << "Moving arm to home pose for safe mode" << endl;
//		if(!elbowCheck(HOME_POSE, rightArm) && !elbowCheck(checked, rightArm))
//		{
			if(rightArm)
			{
#ifdef RIGHT_ARM
				if(moveHand)
				{
					cout << "Attempting to move the whole right arm" << endl;
					rightArmPos->positionMove(HOME_POSE);
				}
				else
				{
					cout << "Attempting to move just the upper right arm" << endl;
					for(int i=0; i<8; i++)
					{
						if(params.m_ROBOT.compare("icubSim")==0 && i==7)
							break;
						if(i!=6)
						{
//							cout << "Attempting to move j" << i << " to " << HOME_POSE[i] << endl;
							rightArmPos->positionMove(i, HOME_POSE[i]);
						}
					}

				}
				do{
					Time::delay(0.2);
					armMoving = true;
//					cout << "Arms moving" << endl;
				}while(!armsStationary());
				rightElbowRetracted = true;
				armMoving = false;
#else
				cout << "Undefined arm requested to move" << endl;
#endif

			}
			else
			{
#ifdef LEFT_ARM
				if(moveHand)
				{
					leftArmPos->positionMove(HOME_POSE);
				}
				else
				{
					for(int i=0; i<8; i++)
					{
						if(params.m_ROBOT.compare("icubSim")==0 && i==7)
							break;
						if(i!=6)
							leftArmPos->positionMove(i, HOME_POSE[i]);
					}
				}
				do{
					Time::delay(0.2);
					armMoving = true;
				}while(!armsStationary());
				leftElbowRetracted = true;
				armMoving = false;
#else
				cout << "Undefined arm requested to move" << endl;
#endif
			}
//		}
	}

	cout << "Moving to position: " << endl;
	for (int i=0; i<16; i++)
	{
		cout << " " << checked[i];
	}
	cout << endl;

//	if(!elbowCheck(checked,rightArm))
//	{
		if(rightArm)
		{
#ifdef RIGHT_ARM
			if(moveHand)
				rightArmPos->positionMove(checked);
			else
			{
				for(int i=0; i<8; i++)
				{
					if(i!=6)
						rightArmPos->positionMove(i, checked[i]);
				}
			}
			rightElbowRetracted = false;
#endif
		}
		else
		{
#ifdef LEFT_ARM
			if(moveHand)
				leftArmPos->positionMove(checked);
			else
			{
				for(int i=0; i<8; i++)
				{
					if(i!=6)
						leftArmPos->positionMove(i, checked[i]);
				}
			}
			leftElbowRetracted = false;
#endif
		}
//	}
	do{
		Time::delay(0.2);
		armMoving = true;
	}while(!armsStationary());
	armMoving = false;


//	if(safeMode)
//	{
//
//		if(rightArm)
//		{
//			rightArmPos->positionMove(elbow, reserve);
//			rightElbowRetracted = false;
//		}
//		else
//		{
//			leftArmPos->positionMove(elbow, reserve);
//			leftElbowRetracted = false;
//		}
//
//		while(!armsStationary() && block)
//		{
//			Time::delay(0.2);
//			armMoving = true;
//		}
//		armMoving = false;
//	}




}

/**
 * EMERGENCY STOP ONLY!
 */
void armController::stopMovement()		//Causes the motors to be disabled!
{
	bool success= true;
#ifdef LEFT_ARM
	success = leftArmPos->stop();
#endif
//	success &= rightArmPos->stop();
	cout << "Stopping movement" << endl;
#ifdef RIGHT_ARM
	if(!board)
		success &= rightArmPos->stop();
#endif
	if(success)
		armMoving = false;
}




bool armController::initYarp()
{
	yarp::os::Network yarp;

	//Check drivers have been correctly established
	bool ok=true;

	string limb = "/";
	limb += params.m_ROBOT;
	string fullLimb = limb + "/right_arm";

	//options.put("remote", "/icub/head");

	Property rightArmOptions;
	string hport = "/smc";
	if(params.m_ROBOT.compare("icubSim")==0)
		hport += "Sim/";
	else hport +="/";
	string fullLocal;
#ifdef RIGHT_ARM

	fullLocal = hport + "right_arm_move";
	rightArmOptions.put("part", "right_arm");
	rightArmOptions.put("device", "remote_controlboard");
	rightArmOptions.put("local", fullLocal.c_str());
	rightArmOptions.put("remote", fullLimb.c_str());

	rightArmDriver=new PolyDriver;
	rightArmDriver->open(rightArmOptions);

	ok &= rightArmDriver->isValid();
#endif

	Property leftArmOptions;
#ifdef LEFT_ARM
	fullLocal = hport + "left_arm_move";
	fullLimb = limb + "/left_arm";
	leftArmOptions.put("part", "left_arm");
	leftArmOptions.put("device", "remote_controlboard");
	leftArmOptions.put("local", fullLocal.c_str());
	leftArmOptions.put("remote", fullLimb.c_str());

	leftArmDriver=new PolyDriver;
	leftArmDriver->open(leftArmOptions);

	ok &= leftArmDriver->isValid();
#endif


	if (!ok)
	{
		cout << "Failed to open one or both of the arm motor drivers\n";
		return ok;
	}


	//Obtain position and encoder interfaces to drivers

#ifdef RIGHT_ARM
	ok &= rightArmDriver->view(rightArmPos);
	ok &= rightArmDriver->view(rightArmEnc);
#endif
#ifdef LEFT_ARM
	ok = leftArmDriver->view(leftArmPos);
	ok &= leftArmDriver->view(leftArmEnc);
#endif

	if (!ok)
	{
		cout << "Failed to obtain one or more of the arm position and encoder interfaces\n";
		return ok;
	}

#ifdef RIGHT_ARM
	rightArmPos->setRefSpeeds(ARM_VELOCITY);
#endif
#ifdef LEFT_ARM
	leftArmPos->setRefSpeeds(ARM_VELOCITY);
#endif

	return ok;
}

/**
 * Compares the arm positions to previous positions after a 1 second delay
 *
 */
bool armController::armsStationary()
{

	bool leftmotionCompleted = true;
	bool rightmotionCompleted = true;

	bool joint;
	for(int i=0; i<6; i++)
	{
#ifdef RIGHT_ARM
		joint=true;
		rightArmPos->checkMotionDone(i, &joint);
		rightmotionCompleted &=joint;
#endif
#ifdef LEFT_ARM
		joint=true;
		leftArmPos->checkMotionDone(i, &joint);
		leftmotionCompleted &=joint;
#endif
	}
//	cout << "Check motion done says left arm motion is ";
	if(leftmotionCompleted && rightmotionCompleted)
	{
		if(moveCalled)
		{
			cout << "arm motion done" << endl;
			moveCalled = false;
		}
		return true;
	}
	else
	{
//		cout << "still moving" << endl;
		return false;
	}


//
//	double* posLeft1 = new double[16];
//	double* posRight1 = new double[16];
//	bool success = true;
//	success &= getCurrentPosition(posLeft1, false);
//	if(!board)
//	{
//		success &= getCurrentPosition(posRight1, true);
//	}
//
//	Time::delay(0.8);
//
//	leftArmPos->checkMotionDone(&motionCompleted);
//	cout << "Check motion done says left arm motion is ";
//	if(motionCompleted)
//		cout << "done";
//	else
//		cout << "still going";
//	cout << endl;
//
//	double* posLeft2 = new double[16];
//	double* posRight2 = new double[16];
//	success &= getCurrentPosition(posLeft2, false);
//	if(!board)
//	{
//		success &= getCurrentPosition(posRight2, true);
//	}
//
//	if(!success)
//	{
//		cout << "Failed to obtain at least one arm configuration" << endl;
//		return false;
//	}
//
//	if(!board && !(isAtPosition(posLeft1, posLeft2) && isAtPosition(posRight1, posRight2)))
//	{
//		cout << "Arms are moving" << endl;
//		return false;
//	}
//	else if(!(isAtPosition(posLeft1, posLeft2) && board))
//	{
//		cout << "Arm is moving" << endl;
//		return false;
//	}
//	else
//	{
//		cout << "Arms are stationary" << endl;
//		return true;
//	}
}


void armController::generateRandomPosition(double* reach, const int noJoints)
{
//	for(int i=0; i<16; i++)
//	{
//		reach[i] = ARM_REST[i];
//	}
//
//	int joints = min(noJoints, 6);
//
//	for(int i=0; i<joints; i++)
//	{
//		int range = maxJointRanges[i] - minJointRanges[i];
//		int randX = randGenerator(range) + minJointRanges[i];
//		reach[i] = (double)randX;
//	}

	static int poseCounter = 0;

	int pos = poseCounter;
//	int pos = randGenerator(NUM_POSES);
	cout << pos << endl;

	for (int i=0; i<16; i++)
	{
		reach[i] = ARM_POSES[pos][i];
	}
	cout << endl;

	poseCounter++;
	poseCounter=poseCounter%NUM_POSES;





}

void armController::generateSmallRelativeMovement(const double* currentPos, double* reach, const int noJoints, int maxRange)
{
	for(int i=0; i<noJoints; i++)
	{
		int change = randGenerator(maxRange*2)-maxRange;
		reach[i] = currentPos[i] + change;
		if(reach[i] > maxJointRanges[i])
		{
			reach[i] = maxJointRanges[i];
		}
		else if(reach[i] < minJointRanges[i])
		{
			reach[i] = minJointRanges[i];
		}
	}
	for(int i=noJoints; i<16; i++)
	{
		reach[i] = currentPos[i];
	}
}



bool armController::babble(const int arm)
{
	double* armPos = new double[16];
	generateRandomPosition(armPos, 4);
	bool rightArm;
	if(arm == 0)
	{
		rightArm = false;
	}
	else if(arm==1)
	{
		rightArm = true;
	}
//	else
//	{
//		if(randGenerator(50)>25)
//			rightArm = true;
//		else
//			rightArm = false;
//
////		move(ARM_REST,false, !rightArm);	// only move other arm if moving both, so don't block
//	}

	move(armPos,true, rightArm);

	return rightArm;
}

bool armController::smallBabble(const bool arm, int maxRange)
{

	double* currentPos = new double[16];
	bool success = getCurrentPosition(currentPos, arm);

	if(success)
	{
		double* nextPos = new double[16];
		generateSmallRelativeMovement(currentPos, nextPos, 4, maxRange);
		move(nextPos,true, arm);
	}


	return success;
	// This method is used after the eye+head has located the hand, and is then regarding it
	// as it makes small movements, to learn links in a local area.


	//make a small arm movement, e.g. max motor range of 5 for each motor?
	//check that the movement is within range
}



void armController::pressButton(const bool rightArm)
{
	double current, pressed;
	if(rightArm)
	{
#ifdef RIGHT_ARM
		rightArmEnc->getEncoder(wristPitch, &current);
		if(current>-10)
			pressed = 0;
		else
			pressed = current+10;
		rightArmPos->positionMove(wristPitch, pressed);
		Time::delay(1);

		rightArmPos->positionMove(wristPitch, current);
#endif
	}
	else
	{
#ifdef LEFT_ARM
		leftArmEnc->getEncoder(wristPitch, &current);
		if(current>-10)
			pressed = 0;
		else
			pressed = current+10;
		leftArmPos->positionMove(wristPitch, pressed);
		Time::delay(1);
		leftArmPos->positionMove(wristPitch, current);
#endif
	}
	Time::delay(0.5);
}


void armController::smallPush(const bool rightArm)
{
	//rotate J4 -80 degrees, or as far as is safe.
	//use button press action
	//return J4 to previous position.

	double current, rotated;
	if(rightArm)
	{
#ifdef RIGHT_ARM
		rightArmEnc->getEncoder(wristProSup, &current);
		if(current<0)
			rotated = -80;
		else
			rotated = current-80;
		rightArmPos->positionMove(wristProSup, rotated);
		while(!armsStationary())
			Time::delay(0.2);
		pressButton(rightArm);
		rightArmPos->positionMove(wristProSup, current);
		while(!armsStationary())
			Time::delay(0.2);
#endif
	}
	else
	{
#ifdef LEFT_ARM
		leftArmEnc->getEncoder(wristProSup, &current);
		if(current<0)
			rotated = -80;
		else
			rotated = current-80;
		leftArmPos->positionMove(wristProSup, rotated);
		while(!armsStationary())
			Time::delay(0.2);
		pressButton(rightArm);
		leftArmPos->positionMove(wristProSup, current);
		while(!armsStationary())
			Time::delay(0.2);
#endif
	}
	Time::delay(0.5);
}



bool armController::getCurrentPosition(double* position, const bool rightArm)
{
	bool success;
	if(rightArm)
	{
#ifdef RIGHT_ARM
		success = rightArmEnc->getEncoders(position);
#endif
	}
	else
	{
#ifdef LEFT_ARM
		success = leftArmEnc->getEncoders(position);
#endif
	}

	return success;
}



bool armController::isAtRest(const bool rightArm)
{
	if(board)
		return isAtPosition(ARM_BOARD_REST, rightArm);
	else
		return isAtPosition(ARM_REST, rightArm);
}


//Note, this will also return false if it was unable to obtain the current arm configuration
bool armController::isAtPosition(const double* compPos, const bool rightArm)
{
	double* currentPos = new double[16];
	bool success = getCurrentPosition(currentPos, rightArm);
	if(success)
		return isAtPosition(currentPos, compPos);
	else
		return success;
}



bool armController::isAtPosition(const double* pos1, const double* pos2)
{
	bool match = true;
	for(int i=0; i<6; i++)
	{
		if(!((floor(pos1[i])==(int)pos2[i]) || (ceil(pos1[i]) == (int)pos2[i])))
		{
			match = false;
			break;
		}

	}
	return match;
}



void armController::armAction(int action, const double* position, bool rightArm)
{
	//Check if a thread is currently executing, if so interrupt

	if(threads.size())
		threads.interrupt_all();

	switch(action)
	{
		case REACH:
			threads.create_thread(boost::bind(&armController::reachRight,this,position));
			break;
		case REACH_AND_PRESS:
			threads.create_thread(boost::bind(&armController::reachAndPressLeft,this,position));
			break;
		case PRESS:
			threads.create_thread(boost::bind(&armController::pressButton,this,rightArm));
			break;
		case SMALL_PUSH:
			threads.create_thread(boost::bind(&armController::smallPush,this,rightArm));
			break;
		case REST:
			threads.create_thread(boost::bind(&armController::armRest,this,rightArm));
			break;
		case REACH_AND_PRESS_LEFT:
			threads.create_thread(boost::bind(&armController::reachAndPressLeft,this,position));
			break;
		case REACH_AND_PRESS_RIGHT:
			threads.create_thread(boost::bind(&armController::reachAndPressRight,this,position));
			break;
		case REACH_AND_PUSH_LEFT:
			threads.create_thread(boost::bind(&armController::reachAndPushLeft,this,position));
			break;
		case REACH_AND_PUSH_RIGHT:
			threads.create_thread(boost::bind(&armController::reachAndPushRight,this,position));
			break;
		case REACH_LEFT:
			threads.create_thread(boost::bind(&armController::reachLeft,this,position));
			break;
		case REACH_RIGHT:
			threads.create_thread(boost::bind(&armController::reachRight,this,position));
			break;

		default:
			cout << "Unrecognised action" << endl;
			break;
	}

}


void armController::reachAndPressLeft(const double* position)
{
	move(position, true, LEFT);
	Time::delay(0.3);
	pressButton(LEFT);

	return;
}

void armController::reachAndPressRight(const double* position)
{
	move(position, true, RIGHT);
	Time::delay(0.3);
	pressButton(RIGHT);

	return;
}

void armController::reachAndPushLeft(const double* position)
{
	move(position, true, RIGHT);
	Time::delay(0.3);
	smallPush(RIGHT);

	return;
}

void armController::reachAndPushRight(const double* position)
{
	move(position, true, LEFT);
	Time::delay(0.3);
	smallPush(LEFT);

	return;
}


void armController::reachLeft(const double* position)
{
	move(position, true, LEFT);

	return;
}

void armController::reachRight(const double* position)
{
	move(position, true, RIGHT);

	return;
}



//Currently Pseudo calculated using the flexion and rotation joints.
//Note, left arm angle should be inverted.
double armController::getDepth(bool rightArm)
{
	double* currentPos = new double[16];
	getCurrentPosition(currentPos, rightArm);

	return currentPos[shoulderRoll] + currentPos[elbow] + currentPos[wristPitch];

}
double armController::getAngle(bool rightArm)
{
	double* currentPos = new double[16];
	getCurrentPosition(currentPos, rightArm);
	double rotation = currentPos[shoulderPitch] + currentPos[shoulderYaw] + currentPos[wristProSup];
	if(!rightArm)
		rotation*=-1;
	return rotation;
}


bool armController::getStretch(double* j1, double* j3, bool rightArm)
{
	double* currentPos = new double[16];
	getCurrentPosition(currentPos, rightArm);

	*j1 = sqrt(401-40*cos(toRadians(currentPos[1])));
	*j3 = sqrt(401-40*cos(toRadians(currentPos[3])));

	return true;
}


double* armController::getMaxJointRanges()
{
    return maxJointRanges;
}

double* armController::getMinJointRanges()
{
    return minJointRanges;
}

void armController::setMaxJointRanges(double* maxJointRanges)
{
    this->maxJointRanges = maxJointRanges;
}

void armController::setMinJointRanges(double* minJointRanges)
{
    this->minJointRanges = minJointRanges;
}


///**
// * Returns a psudo random number between 0 and range (not including range)
// */
//int armReach::randGenerator(const int range)
//{
//	if(range==0)
//		return 0;
//	return rand() % range;
//}


//Hand regard - make small random movement

//future extensions:
//development of touch?
//calc distance from body?



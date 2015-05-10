/*
 * graspController.cpp
 *
 *  Created on: Dec 1, 2011
 *      Author: icub
 *
 *
 *      Index finger: 	0-11
 *      Second finger:  12-23
 *      Third finger:	24-35
 *      Little finger:	36-47
 *      Thumb:			48-59
 *      Palm:			~96-143?
 *
 *
 */
#include "graspController.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

graspController::graspController(string robot, armController* arm, bool useSkin)
{
	//Calls constructor on armController first... grrrrr
//	board=false;

	skin = useSkin;

//	initYarp();
	ac = arm;
	initGrasp(robot);
	grasping=false;
#ifdef RIGHT_ARM
	release(true);	//rightarm
#endif
#ifdef LEFT_ARM
	release(false);	//leftarm
#endif
}

bool graspController::initGrasp(string robot)
{
	Network yarp;

	string skinny = "/";
	skinny += robot;
	skinny += "/skin/";
	//options.put("remote", "/icub/head");

	string fullSkinny = skinny + "righthand";
#ifdef RIGHT_ARM
	portRight.open("/skinTest/rightHand");
	yarp.connect(fullSkinny.c_str(), "/skinTest/rightHand");
#endif

	fullSkinny = skinny + "lefthand";
#ifdef LEFT_ARM
	portLeft.open("/skinTest/leftHand");
	yarp.connect(fullSkinny.c_str(), "/skinTest/leftHand");
#endif

	if(skin)
	{


	Bottle *inputRight = NULL;
	Bottle *inputLeft = NULL;
#ifdef RIGHT_ARM
	inputRight = portRight.read();
#endif
#ifdef LEFT_ARM
	inputLeft= portLeft.read();
#endif



	//collect skin data for some time, and compute the 95% percentile
	float skinRight[SAMPLES][SENSORS];
	float skinLeft[SAMPLES][SENSORS];
	float varRight[SENSORS];
	float varLeft[SENSORS];


	//collect data
	for (int i=0; i<SAMPLES; i++) {
#ifdef RIGHT_ARM
		inputRight = portRight.read();
#endif
#ifdef LEFT_ARM
		inputLeft= portLeft.read();
#endif

		if (inputRight!=NULL) {
			for (int j=0; j<SENSORS; j++) {
                skinRight[i][j] = (float)(inputRight->get(j).asDouble());
			}
		}
		if (inputLeft!=NULL) {
			for (int j=0; j<SENSORS; j++) {
                skinLeft[i][j] = (float)(inputLeft->get(j).asDouble());

			}
		}
		Time::delay(0.05);
	}

	//calc averages
	for(int i=0; i<SENSORS; i++)
	{
		avgRight[i] = 0;
		avgLeft[i] = 0;

		maxDiffRight[i] = 0;
		maxDiffLeft[i]=0;

		varRight[i] = 0;
		varLeft[i] = 0;
	}
	for(int i=0; i<SAMPLES; i++)
	{
		for(int j=0; j<SENSORS; j++)
		{
			avgRight[j] += skinRight[i][j];
			avgLeft[j] += skinLeft[i][j];
		}
	}

	string right = "Right: ";
	string left = "Left: ";
	string diffR = "Max diff right: ";
	string diffL = "Max diff left: ";

	//calc diff from mean and square
	float diffRight[SAMPLES][SENSORS];
	float diffLeft[SAMPLES][SENSORS];

	for(int j=0; j<SENSORS; j++)
	{
		avgRight[j] /= SAMPLES;
		avgLeft[j] /= SAMPLES;

		for(int i=0; i<SAMPLES; i++)
		{
			diffRight[i][j] = pow(skinRight[i][j] - avgRight[j], 2);
			diffLeft[i][j] = pow(skinLeft[i][j] - avgLeft[j], 2);
#ifdef RIGHT_ARM
			if((avgRight[j]-skinRight[i][j])>maxDiffRight[j])
				maxDiffRight[j] = avgRight[j]-skinRight[i][j];
#endif
#ifdef LEFT_ARM
			if((avgLeft[j]-skinLeft[i][j])>maxDiffLeft[j])
				maxDiffLeft[j] = avgLeft[j]-skinLeft[i][j];
#endif
		}

#ifdef RIGHT_ARM
		right= appendInt(right, avgRight[j]);
		right += " ";
		diffR= appendInt(diffR, maxDiffRight[j]);
		diffR += " ";
#endif
#ifdef LEFT_ARM
		left= appendInt(left, avgLeft[j]);
		left += " ";
		diffL= appendInt(diffL, maxDiffLeft[j]);
		diffL += " ";
#endif
	}

	cout << "Averages:" << endl;
#ifdef RIGHT_ARM
	cout << "   " << right << endl;
#endif
#ifdef LEFT_ARM
	cout << "   " << left  << endl;
#endif

	cout << "Maximum differences:" << endl;
#ifdef RIGHT_ARM
	cout << "   " << diffR << endl;
#endif
#ifdef LEFT_ARM
	cout << "   " << diffL << endl;
#endif


	//calc variance (average of squared differences)
	for(int i=0; i<SAMPLES; i++)
	{
		for(int j=0; j<SENSORS; j++)
		{
			varRight[j] += diffRight[i][j];
			varLeft[j] += diffLeft[i][j];
//			printf("left: %i %f\n", j, varLeft[j]);
		}
	}
	right = "Right: ";
	left = "Left: ";

	//calc standard deviation (square root of the variance)
	for(int j=0; j<SENSORS; j++)
	{
		varRight[j] /= SAMPLES;
		varLeft[j] /= SAMPLES;

		sdRight[j] = sqrt(varRight[j]);
		sdLeft[j] = sqrt(varLeft[j]);

		right = appendInt(right, sdRight[j]);
		right += " ";
		left = appendInt(left, sdLeft[j]);
		left += " ";
	}

	cout << "Standard deviation:" << endl;
#ifdef RIGHT_ARM
	cout << "   " << right << endl;
#endif
#ifdef LEFT_ARM
	cout << "   " << left  << endl;
#endif

	}
//	delete skinLeft;
//	delete skinRight;
//	delete varLeft;
//	delete varRight;

}

void graspController::closePorts()
{
	portRight.close();
	portLeft.close();
}


bool graspController::handsStationary()
{
	bool success,flag;
	for(int i=7; i<16;i++)
	{
#ifdef LEFT_ARM
		success = ac->leftArmPos->checkMotionDone(i, &flag);
		if(!(success &&flag))
			return false;
#endif
#ifdef RIGHT_ARM
//		if(i!=14)
//		{
			success = ac->rightArmPos->checkMotionDone(i, &flag);
			if(!(success &&flag))
				return false;
//		}
#endif
	}
	return true;
}

bool graspController:: move(const double* position,bool block, bool rightArm)
{
//	cout << "Desired position: " << endl;
//	for (int i=5; i<16; i++)
//	{
//		cout << " " << position[i];
//	}
//	cout << endl;

//	double* checked = new double[16];
	double checked[16];
	for(int i=8; i<16;i++)
	{
		if(position[i] < absMinJointRanges[i])
			checked[i] = absMinJointRanges[i];
		else if(position[i] > absMaxJointRanges[i])
			checked[i] = absMaxJointRanges[i];
		else
			checked[i] = position[i];


		if(rightArm)
		{
#ifdef RIGHT_ARM
			ac->rightArmPos->positionMove(i, checked[i]);
#endif
		}
		else
		{
#ifdef LEFT_ARM
			ac->leftArmPos->positionMove(i, checked[i]);
#endif
		}
	}

	cout << "Moving to position: ";
	for (int i=8; i<16; i++)
	{
		printf(" %.2f", checked[i]);
//		cout << " " << checked[i];
	}
	cout << endl;
	bool timeOut=false;
	time_t startSeconds;
	startSeconds = time(NULL);
	time_t timeTaken = startSeconds-startSeconds;
	while(block && !handsStationary())
	{
		time_t current = time(NULL);
		timeTaken=current - startSeconds;
		if(timeTaken > 30)
		{
			timeOut = true;
			cout << "Time out on grasp action" << endl;
			break;
		}


		Time::delay(0.2);
	}
	return !timeOut;
}

//TODO: Make this function into a thread, this can then be interrupted by input
bool graspController::grasp(bool rightArm)
{
	//update tactile data
	float currentReading[SENSORS];

	bool grasped=false;
	bool empty=false;
	bool thumb=false, index=false, second=false, pinky=false, palm=false;
	bool tEmpty=false, iEmpty=false, sEmpty=false, pEmpty=false;
	double* startArmPose = new double[16];
	double* currentArmPose = new double[16];
	double* previousArmPose = new double[16];
	double* newArmPose = new double[16];
	bool success;

	int safetyThumb=0, safetyIndex=0, safetySecond=0, safetyPinky=0;
	double pnky;

	if(rightArm)
	{
#ifdef RIGHT_ARM
		ac->rightArmPos->positionMove(9, 5);
		ac->rightArmPos->positionMove(10, 5);
#endif
	}
	else
	{
#ifdef LEFT_ARM
		ac->leftArmPos->positionMove(9, 5);
		ac->leftArmPos->positionMove(10, 5);
#endif
	}
	do
	{
		Time::delay(0.5);
	}while(!handsStationary());


	ac->getCurrentPosition(startArmPose,rightArm);
	success = ac->getCurrentPosition(currentArmPose,rightArm);

	if(success)
	{
		 currentArmPose[8] = 70;	//55
		 if(currentArmPose[15]<20)
			 currentArmPose[15] = 20;
		 pnky=20;
		 move(currentArmPose,true,rightArm);
		 Time::delay(1);
	}
	else
	{
		printf("Didn't get current position\n");
		return false;
	}

	for(int i=0; i<16; i++)
	{
		previousArmPose[i] = 0;
		newArmPose[i]=currentArmPose[i];
	}

	float avg[SENSORS];
	float sd[SENSORS];
	float maxDiff[SENSORS];

	Time::delay(1);

	int counter = 0;
	int gTest = 0;
	int eTest = 0;

	while(!grasped && !empty && (gTest+eTest)<5 && counter<30)
	{
		counter ++;
		if(rightArm){
			if(data=portRight.read(0)){
				for(int i=0; i<SENSORS; i++)
				{
					currentReading[i] = (float)(data->get(i).asDouble());
					avg[i] = avgRight[i];
					sd[i] = sdRight[i];
					maxDiff[i] = maxDiffRight[i];
				}
			}

		}
		else{
			if(data=portLeft.read(0)){
				for(int i=0; i<SENSORS; i++)
				{
					currentReading[i] = (float)(data->get(i).asDouble());
					avg[i] = avgLeft[i];
					sd[i] = sdLeft[i];
					maxDiff[i] = maxDiffLeft[i];
				}
			}
		}


		double pressure = 0;
		for(int i=0; i<12; i++)
		{

			if((avg[i]-currentReading[i]-maxDiff[i]) > (sd[i] + TOUCH_THRESHOLD))
			{
				index = true;
				pressure += (avg[i]-currentReading[i]);
				break;

			}
			index = false;
		}
		if(index)
		{
			printf("Index contact pressure: %.2f\n", pressure);
		}

		pressure = 0;
		for(int i=12; i<24; i++)
		{
			if((avg[i]-currentReading[i]-maxDiff[i]) > (sd[i] + TOUCH_THRESHOLD))
			{
				second = true;
				pressure += (avg[i]-currentReading[i]);
				break;
			}
			second = false;
		}
		if(second)
		{
			printf("Second contact pressure: %.2f\n", pressure);
		}


		pressure = 0;
		for(int i=24; i<48; i++)
		{
			if((avg[i]-currentReading[i]-maxDiff[i]) > (sd[i] + TOUCH_THRESHOLD))
			{
				pinky = true;
				pressure += (avg[i]-currentReading[i]);
				break;
			}
			pinky = false;
		}
		if(pinky)
		{
			printf("Ring and Pinky contact pressure: %.2f\n", pressure/2);
		}


		pressure = 0;
		for(int i=48; i<60; i++)
		{
			if((avg[i]-currentReading[i]-maxDiff[i]) > (sd[i] + TOUCH_THRESHOLD))
			{
				thumb = true;
				pressure += (avg[i]-currentReading[i]);
				break;
			}
			thumb = false;
		}
		if(thumb)
		{
			printf("Thumb contact pressure: %.2f\n", pressure);
		}


//		pressure = 0;
//		for(int i=60; i<144; i++)
//		{
//			if((avg[i]-currentReading[i]-maxDiff[i]) > (sd[i] + TOUCH_THRESHOLD))
//			{
//				palm = true;
//				pressure += (avg[i]-currentReading[i]);
//
//			}
//		}
//		if(palm)
//		{
//			printf("Palm contact pressure: %.2f\n", pressure);
//		}


		success = ac->getCurrentPosition(currentArmPose,rightArm);
		if(!success)
			continue;


		if(!thumb)
		{
			//Check if not moved from previous position
			if((currentArmPose[9]<= previousArmPose[9] ||
				currentArmPose[10]<= previousArmPose[10]))
			{
				if(safetyThumb>=10)
				{
					printf("Thumb Safety\n");
					thumb=true;
				}
				if((int)currentArmPose[9]==(int)startArmPose[9] || (int)currentArmPose[10]==(int)startArmPose[10])
				{
					newArmPose[9] = currentArmPose[9] + 5;
					newArmPose[10] = currentArmPose[10] + 5;
				}
				else if(currentArmPose[9]<10 || currentArmPose[10]<10)
				{
					safetyThumb++;
				}
				//Thumb not moved
				else if(currentArmPose[9]<35 || currentArmPose[10]<35)
				{
					newArmPose[9] = currentArmPose[9];
					newArmPose[10] = currentArmPose[10];
					printf("Thumb stopped moving: %.2f vs %.2f and %.2f vs %.2f\n", currentArmPose[9], previousArmPose[9], currentArmPose[10], previousArmPose[10]);
					thumb=true;
				}
			}
			else		//check if safety limit reached
			{
				bool baa1 =false;
				bool baa2 =false;
				if(currentArmPose[9]<35)
				{
					newArmPose[9] = currentArmPose[9] + 5;
					printf("Thumb moving\n");
				}
				else
				{
					printf("Thumb reached threshold\n");
					newArmPose[9] = currentArmPose[9];
					baa1=true;
				}
				if(currentArmPose[10]<35)
				{
					newArmPose[10] = currentArmPose[10] + 5;
					printf("Thumb moving\n");
				}
				else
				{
					newArmPose[10] = currentArmPose[10];
					printf("Thumb reached threshold\n");
					baa2=true;
				}
				tEmpty = baa1 && baa2;
			}

		}


		if(!index)
		{
			//Check if not moved from previous position
			if((currentArmPose[11]<= previousArmPose[11] ||
				currentArmPose[12]<= previousArmPose[12]) )
			{
				if(safetyIndex>=10)
				{
					printf("Index Safety\n");
					index=true;
				}
				if((int)currentArmPose[11]==(int)startArmPose[11] || (int)currentArmPose[12]==(int)startArmPose[12])
				{
					newArmPose[11] = currentArmPose[11] + 5;
					newArmPose[12] = currentArmPose[12] + 5;
				}
				else if(currentArmPose[11]<10 || currentArmPose[12]<10)
				{
					safetyIndex++;
				}
				//index not moved
				else if(currentArmPose[11]<35 || currentArmPose[12]<35)
				{
					index=true;
					newArmPose[11] = currentArmPose[11];
					newArmPose[12] = currentArmPose[12];
					printf("Index stopped moving: %.2f vs %.2f and %.2f vs %.2f\n", currentArmPose[11], previousArmPose[11], currentArmPose[12], previousArmPose[12]);
				}
			}
			else		//check if safety limit reached
			{
				bool baa1 =false;
				bool baa2 =false;
				if(currentArmPose[11]<35)
				{
					newArmPose[11] = currentArmPose[11] + 5;
					printf("Index moving\n");
				}
				else
				{
					newArmPose[11] = currentArmPose[11];
					baa1=true;
					printf("Index reached threshold\n");
				}
				if(currentArmPose[12]<35)
				{
					newArmPose[12] = currentArmPose[12] + 5;
					printf("Index moving\n");
				}
				else
				{
					newArmPose[12] = currentArmPose[12];
					baa2=true;
					printf("Index reached threshold\n");
				}
				iEmpty = baa1 && baa2;
			}
		}

		if(!second)
		{
			//Check if not moved from previous position
			if((currentArmPose[13]<= previousArmPose[13] ||
				currentArmPose[14]<= previousArmPose[14]))
			{
				if(safetySecond>=10)
				{
					printf("Second Safety\n");
					second=true;
				}
				if((int)currentArmPose[13]==(int)startArmPose[13] || (int)currentArmPose[14]==(int)startArmPose[14])
				{
					newArmPose[13] = currentArmPose[13] + 5;
					newArmPose[14] = currentArmPose[14] + 5;
				}
				else if(currentArmPose[13]<10 || currentArmPose[14]<10)
				{
					safetySecond++;
				}
				//second not moved
				else if(currentArmPose[13]<35 || currentArmPose[14]<35)
				{
					newArmPose[13] = currentArmPose[13];
					newArmPose[14] = currentArmPose[14];
					second=true;
					printf("Second stopped moving: %.2f vs %.2f and %.2f vs %.2f\n", currentArmPose[13], previousArmPose[13], currentArmPose[14], previousArmPose[14]);
				}
			}
			else		//check if safety limit reached
			{
				bool baa1 =false;
				bool baa2 =false;
				if(currentArmPose[13]<35)
				{
					newArmPose[13] = currentArmPose[13] + 5;
					printf("Second moving\n");
				}
				else
				{
					newArmPose[13] = currentArmPose[13];
					baa1=true;
					printf("Second reached threshold\n");
				}
				if(currentArmPose[14]<35)
				{
					newArmPose[14] = currentArmPose[14] + 5;
					printf("Second moving\n");
				}
				else
				{
					newArmPose[14] = currentArmPose[14];
					baa2=true;
					printf("Second reached threshold\n");
				}
				sEmpty = baa1 && baa2;
			}
		}


		if(!pinky)
		{

			//Check if not moved from previous position
			if((currentArmPose[15]<= previousArmPose[15]))
			{
				if(safetyPinky>=10)
				{
					printf("Pinky Safety\n");
					pinky=true;
				}
				if((int)currentArmPose[15]==(int)startArmPose[15])
				{
					newArmPose[15] = currentArmPose[15] + 10;
				}
				else if(currentArmPose[15]<30)
				{
					safetyPinky++;
				}
				//second not moved
				else if(currentArmPose[15]<90)
				{
					newArmPose[15] = currentArmPose[15];
					pinky=true;
					printf("Pinky stopped moving: %.2f vs %.2f\n", currentArmPose[15], previousArmPose[15]);
				}
			}
			else		//check if safety limit reached
			{
				if(currentArmPose[15]<90)
				{
					pnky+=10;
					newArmPose[15] =  pnky;
					printf("Pinky moving\n");
				}
				else
				{

					newArmPose[15] = currentArmPose[15];
					pEmpty=true;
					printf("pinky reached threshold\n");
				}
			}
		}

		for(int i=0; i<16; i++)
		{
			previousArmPose[i] = currentArmPose[i];
		}

		move(newArmPose,true,rightArm);
		Time::delay(0.3);

		if(thumb && (index || second || pinky))
			grasped = true;

		empty = tEmpty && iEmpty && sEmpty && pEmpty;


		gTest = thumb + index+ second + pinky;
		eTest = tEmpty + iEmpty + sEmpty + pEmpty;

		printf("Grasp Count: %i,  Empty Count: %i, attempt count: %i\n", gTest, eTest, counter);
	}

	grasping = grasped;
	return grasped;

}


bool graspController::release(bool rightArm)
{
	double* currentArmPose = new double[16];
	ac->getCurrentPosition(currentArmPose,rightArm);
	for(int i=9; i<16; i++)
	{
		currentArmPose[i] = 5;
	}
	move(currentArmPose, true, rightArm);
	grasping = false;
	Time::delay(1.5);
	currentArmPose[10] = 32;
//	currentArmPose[9] = 66;
	currentArmPose[8] = 5;
//	move(currentArmPose, true, rightArm);
	return !grasping;
}

bool graspController::fist(bool rightArm)
{
	double* currentArmPose = new double[16];
	ac->getCurrentPosition(currentArmPose,rightArm);

	currentArmPose[8] = 85;
	currentArmPose[9] = 5;
	currentArmPose[10] = 40;
	currentArmPose[11] = 60;
	currentArmPose[12] = 60;
	currentArmPose[13] = 60;
	currentArmPose[14] = 60;
	currentArmPose[15] = 150;

	move(currentArmPose, true, rightArm);

}


bool graspController::openThumb(bool rightArm)
{
	double* currentArmPose = new double[16];
	ac->getCurrentPosition(currentArmPose,rightArm);
	currentArmPose[9] = 5;
	return move(currentArmPose, true, rightArm);
}

bool graspController::closeThumb(bool rightArm)
{
	double* currentArmPose = new double[16];
	ac->getCurrentPosition(currentArmPose,rightArm);
	currentArmPose[9] = 66;
	return move(currentArmPose, true, rightArm);
}

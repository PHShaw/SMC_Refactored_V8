/*
 * headSaccading.cpp
 *
 *  Created on: 8 Jun 2011
 *      Author: icub
 */

#include "headSaccading.h"
using namespace std;
using namespace yarp::os;

headSaccading::headSaccading(headController* pHead, eyeController* pEye, eyeSaccading* pEyeSac, Target* pTarget, ffm* phead_ppm,
		bool pLearn, bool pNeigh, string ppath)
{
	head = pHead;
	eyeSac = pEyeSac;
	eye = pEye;
	target = pTarget;
	nearestNeighbour = pNeigh;	//TODO: used in calcLink

	head_ppm = phead_ppm;

	learn = pLearn;
	path = ppath;

	openLogs();

}

headSaccading::~headSaccading()
{
	// TODO Auto-generated destructor stub
}

bool headSaccading::allStationary()
{
	return head->stationary() && eye->stationary();
}


//Used to record the eye position prior to the eye making a saccade
void headSaccading::recordPrePositions()
{
	while(!allStationary())
		Time::delay(0.1);

	eye->getCurrentPosition(&EyeX1, &EyeY1);
	head->getCurrentPosition(&HeadX1, &HeadY1);

	string entry = "Pre-position of eye is (";
	entry = appendInt(entry, (int)EyeX1);
	entry += ",";
	entry = appendInt(entry, (int)EyeY1);
	entry += ") and head is (";
	entry = appendInt(entry, (int)HeadX1);
	entry += ",";
	entry = appendInt(entry, (int)HeadY1);
	entry += ")";
	logEntry(entry);
}


/**
 * Blocks until the move is completed.
 */
bool headSaccading::gotoField(Field* motorfield)
{
	logEntry("Moving to an existing linked field:");
	logField(motorfield);
	double relX = motorfield->getXcoord();		//relative motor values
	double relY = motorfield->getYcoord();

	//convert to actual motor values (or figure out why relativeMove doesn't work)

	double curX, curY;
	head->getCurrentPosition(&curX, &curY);

	double x, y;
	y = relY+curY;
	x = relX+curX;

	logEntry("Actual movement is ", x ,y);
	head->move(x, y, true);

	return true;
}


bool headSaccading::makeLink(double startX, double startY, double endX, double endY,
								double motorRelX, double motorRelY)
{
	//get field for relative motor movement
	string entry = "Adding a link for start(";
	entry = appendInt(entry, startX);
	entry += ", ";
	entry = appendInt(entry, startY);
	entry += "), end(";
	entry = appendInt(entry, endX);
	entry += ", ";
	entry = appendInt(entry, endY);
	entry += "), relMotor(";
	entry = appendInt(entry, motorRelX);
	entry += ", ";
	entry = appendInt(entry, motorRelY);
	entry += ")";
	logEntry(entry);

	bool gotfield = true;
	PolarField* motor = new PolarField();
	gotfield = isMotorField(motorRelX, motorRelY);

	if(gotfield)
	{
		motor = getMotorField(motorRelX, motorRelY);
		bool ok = makeLink(startX, startY, endX, endY, motor);
		return ok;
	}
	else
		return false;

}


bool headSaccading::makeLink(double startX, double startY, double endX, double endY, PolarField* motor)
{
	double centreX = 320/2;
	double centreY = 240/2;
	double xOffset = endX - centreX;		//Can only use this because the eye should be fixated on target
	double yOffset = endY - centreY;

	double sourceX = startX - xOffset;
	double sourceY = startY - yOffset;

	PolarField* source = new PolarField();

	bool gotfield = isRetinaField(sourceX, sourceY);

	if(gotfield)
	{
		source = getRetinaField(sourceX, sourceY);
		bool ok = makeLink(source,motor);
		return ok;
	}
	else
		return false;
}


/**
 * All parameters are relative values
 */
bool headSaccading::makeLink(double inputX, double inputY, double motorX, double motorY)
{
	PolarField* source = new PolarField();

	bool gotfield = isRetinaField(inputX, inputY);

	if(gotfield)
	{
		source = getRetinaField(inputX, inputY);
		PolarField* motor = new PolarField();
		gotfield = isMotorField(motorX, motorY);

		if(gotfield)
		{
			motor = getMotorField(motorX, motorY);
			bool ok = makeLink(source,motor);
			return  ok;
		}
	}

	return gotfield;
}

bool headSaccading::makeLink(PolarField* source, PolarField* motor)
{
	logEntry("Making a headlink between fields: ");
	logField(source);
	logField(motor);


	if(head_ppm->containsLink(source, motor))
	{
		FieldLink* link = head_ppm->getLink(source, motor);
		link->useField();
		source->useField();
		motor->useField();
		logEntry("strengthening existing link, rather than adding new link");
		return true;
	}

	bool ok = head_ppm->addLink(source, motor);
	if(ok){
		source->linkAdded();
		motor->linkAdded();
		source->setCalcdLinks(100);
		motor->setCalcdLinks(100);
	}
	return ok;
}




bool headSaccading:: calcLink(string colour, vor* v)
{
	while(!allStationary())
		Time::delay(0.1);


	double EyeX3, EyeY3;	//Eye position after head compensation
	eye->getCurrentPosition(&EyeX3, &EyeY3);

	double HeadX2, HeadY2;
	head->getCurrentPosition(&HeadX2, &HeadY2);

	string entry = "Final position of eye is (";
	entry = appendInt(entry, (int)EyeX3);
	entry += ",";
	entry = appendInt(entry, (int)EyeY3);
	entry += ") and head is (";
	entry = appendInt(entry, (int)HeadX2);
	entry += ",";
	entry = appendInt(entry, (int)HeadY2);
	entry += ")";
	logEntry(entry);

	double eyeSaccadeX, eyeSaccadeY;	//magnitude and direction of eye saccade
	eyeSaccadeX = EyeX2 - EyeX1;		//relative motor values for eye saccade
	eyeSaccadeY = EyeY2 - EyeY1;

	double headSaccadeX, headSaccadeY;	//magnitude and direction of head movement
	headSaccadeX = HeadX2 - HeadX1;		//relative motor values for head movement
	headSaccadeY = HeadY2 - HeadY1;

	double eyeOverallX, eyeOverallY;	//Eye Resultant change
	eyeOverallX = EyeX3 - EyeX1;
	eyeOverallY = EyeY3 - EyeY1;

	double eyeCompensationX, eyeCompensationY;
	eyeCompensationX = EyeX3 - EyeX2;
	eyeCompensationY = EyeY3 - EyeY2;
	logEntry ("Eye compensation is ", eyeCompensationX, eyeCompensationY);

	double xOver=0, yOver=0;
	if(v->getOverflow(&xOver, &yOver))
	{
		eyeCompensationX += xOver;
		eyeCompensationY += yOver;
		logEntry("Overflow received is ", xOver, yOver);
	}
	logEntry("Overflow received is ", xOver, yOver);

	bool success;

	PolarField* eyeMotor = new PolarField();
	float dist = 0;
	if(nearestNeighbour)
		eyeMotor = eyeSac->getNearestLearntOutput(eyeCompensationX, eyeCompensationY, &dist);	//isMotorField(eyeCompensationX, eyeCompensationY);
	else
	{
		bool gotfield = eyeSac->isMotorField(eyeCompensationX, eyeCompensationY);
		if(gotfield)
		{
			eyeMotor = eyeSac->getMotorField(eyeCompensationX, eyeCompensationY);
			dist = eyeMotor->getRadius();
		}
		else
		{
			cout << "Head not got field from eye Motor" << endl;
			success = false;
			return success;
		}
	}
	double radius = eyeMotor->getRadius();
	logEntry("Eye Motor field returned:");
	logField(eyeMotor);
	if(dist<(radius*3.5) && dist>0)// && eyeMotor->getUsage()>=1)	//NEW: Force eye motor field to have been proven useful
	{

		logEntry("Distance and usage: ", dist, eyeMotor->getUsage());
//		eyeMotor = eyeSac->getMotorField(eyeCompensationX, eyeCompensationY);
		PolarField* eyeLink = new PolarField();
		eyeLink = eyeSac->getLinkedInput(eyeMotor);

		FieldLink* link = eyeSac->getLink(eyeLink, eyeMotor);

		logEntry("Eye retina field obtained:");
		logField(eyeLink);
		if(link->getUsage()>=1)	//NEW
		{
//		if(gotfield)
//		{
			double targetHeadX = eyeLink->getXcoord();
			double targetHeadY = eyeLink->getYcoord();

			//Learn link from just head movement
			cout << "Attempting to learn link based on head only movement" << endl;
			//head movement inverted to bring target back to centre, i.e. eye compensation
			//Confirm target location and use that for making the link
			double targX, targY;
			bool centred = target->targetCentred(&targX, &targY,colour);
//			if(centred)
//			{
				success = makeLink(targetHeadX, targetHeadY, targX, targY, -headSaccadeX, -headSaccadeY);
				if(success)
				{
					logEntry("link added based on head only movement");
				}
				else
				{
					cout << "Head failed to add link based on head only movement" << endl;
					logEntry("Failed to add link based on head only movement");
				}
//			}
//			else
//			{
//				logEntry("Target not centred so not adding a link");
//
//				success=false;
//			}



		}
		else
		{
			logEntry("Eye Retina field not good enough to trust");
			cout << "Head says Eye Retina field not good enough to trust" << endl;
			success = false;
		}
	}
	else
	{
		logEntry("Eye Motor field not good enough or close enough to trust");
		cout << "Eye Motor field not good enough or close enough to trust" << endl;
		success = false;
	}

	return success;

}




/**
 * Full head movement is equivalent to full eye movement, so can be directly
 * added into the mapping
 */
bool headSaccading:: addFullHeadMovement()
{
	cout << "Attempting to learn link for full head movement" << endl;
//	double relX = EyeX2-EyeX1;
//	double relY = EyeY2-EyeY1;


	double compNewEyetoHeadX, compNewEyetoHeadY, compCurrentEyetoHeadX, compCurrentEyetoHeadY;

	compNewEyetoHeadX = 0.86*EyeX2 - 0.3;
	compCurrentEyetoHeadX = 0.86*EyeX1 - 0.3;

	compNewEyetoHeadY = -0.83*EyeY2 + 0.7;
	compCurrentEyetoHeadY = -0.83*EyeY1 + 0.7;

	logEntry("Pre eye to head compensation is: ", compCurrentEyetoHeadX, compCurrentEyetoHeadY);
	logEntry("Mid eye to head compensation is: ", compNewEyetoHeadX, compNewEyetoHeadY);

	double headRelX, headRelY;
	headRelX = (compNewEyetoHeadX - compCurrentEyetoHeadX) * -1;
	headRelY = (compNewEyetoHeadY - compCurrentEyetoHeadY) * -1;


//	relX = -relX;	//invert to make movement in same direction
	double endTargX, endTargY;
	target->getTarget(&endTargX, &endTargY);
	bool success = makeLink(startTargX, startTargY, endTargX, endTargY, headRelX, headRelY);
	if(success)
	{
		logEntry("Link successfully added for total head movement");
	}
	else
	{
		logEntry("Failed to add link for total head movement");
	}
	return success;

}




/**
 * The makes a random head movement and learns a link.  It does not look for any
 * pre-existing links.
 */
bool headSaccading::saccade(int saccadecounter, double pstargX, double pstargY, string colour, vor* v)
{
	while(!allStationary())
		Time::delay(0.1);

	eye->getCurrentPosition(&EyeX2, &EyeY2);//record the finished eye position before
											// it starts compensating for head movement

	string entry = "Mid-position of eye is (";
	entry = appendInt(entry, (int)EyeX2);
	entry += ",";
	entry = appendInt(entry, (int)EyeY2);
	entry += ")";
	logEntry(entry);


	startTargX = pstargX;
	startTargY = pstargY;

//	if(learn)
//		addFullHeadMovement();	//Full head movement is equivalent to full eye movement

	cout << "making random head movement" << endl;
	double startX, startY;
	head->getCurrentPosition(&startX, &startY);
	head->babble();	//blocks till movement complete

	Time::delay(0.2);
	while(!allStationary())	//but double check anyway, just in case
		Time::delay(0.1);
	double x,y;
	head->getCurrentPosition(&x, &y);

	double relX = x - startX;
	double relY = y - startY;
	motorRecord(relX, relY, saccadecounter);

	logEntry("Made a random movement to actual ", x,y);

	Time::delay(0.5);	// Allow time for VOR to adjust
//	target->getTarget(&targX,&targY);	//Target should be central

	target->getTarget(&targX, &targY, colour);
	eyeSac->autoCenter(targX,targY,colour);
	Time::delay(0.2);
	if(!target->targetCentred(&targX, &targY, colour))
	{
		target->getLastTargs(&targX, &targY);
		double dist;
		if(!target->fovea(targX, targY, &dist))
		{
			cout << "Target not centred?" << endl;
			return false;
		}
	}

//	bool success = true; //calcLink(v);	//just testing basic links at the moment, to try and get that right
	bool success = false;
	if(learn)
		success = calcLink(colour, v);
	return success;
}

//No learning, single movement saccade - no guarantee of fixation
//Aim is that this can be used to centre the eyes in head on target
//So should be dealing with eye position, rather than target location
bool headSaccading::simpleSaccade()
{

	double eyeX, eyeY;
	eye->getCurrentPosition(&eyeX, &eyeY);
	PolarField* eyeMfield = new PolarField();
	bool gotField = eyeSac->isMotorField(eyeX, eyeY);
	if(gotField)
		eyeMfield = eyeSac->getMotorField(eyeX, eyeY);

	PolarField* eyeRfield = new PolarField();
	gotField &= eyeSac->isLinkedInput(eyeMfield);
	if(gotField)
		eyeRfield = eyeSac->getLinkedInput(eyeMfield);
	else
	{
		logEntry("Failed to obtain eye input field for simple head saccade");
		head->babble();
	}

	double targX, targY;
	targX = eyeRfield->getXcoord();
	targY = eyeRfield->getYcoord();

	PolarField* headRfield = new PolarField();
	gotField = isRetinaField(targX, targY);
	if(gotField)
		headRfield = getRetinaField(targX, targY);
	else
	{
		logEntry("Failed to obtain head input field for simple saccade");
		head->babble();
	}

	PolarField* headMfield = new PolarField();
	gotField = isLinkedOutput(headRfield);
	bool success = false;
	if(gotField)
	{
		headMfield = getLinkedOutput(headRfield);
		success = gotoField(headMfield);
	}

	else
	{
		logEntry("Input field  not linked for simple head saccade");
		head->babble();
	}

	if(!success)
	{
		logEntry("Target is not reachable for simple head saccade");
	}

	//colour can be centred without eyes being centred
	//this success measure should be related to eye position
	eye->getCurrentPosition(&eyeX, &eyeY);
	eyeX = abs(eyeX);
	eyeY = abs(eyeY);

	if(eyeX <= 5 && eyeY <= 5)
	{
		return true;
	}
	else
		return false;

}


bool headSaccading::iStyleSaccade(int saccadeCounter, double targX, double targY, std::string colour)
{
	PolarField* startInputField = getRetinaField(targX, targY);
	PolarField* nearbyInput = new PolarField();

	saccadeSteps.clear();
	int failedDirectLinksCounter = 0;
	int failedNeighbourCounter = 0;
	bool followedNeighbour = false;
	int noTargetCounter=0;

	while(!head->stationary())
	{
		Time::delay(0.2);
	}

	double startTargX = targX, startTargY = targY;
	string startColour = colour;

	double startX, startY;
	bool success = head->getCurrentPosition(&startX, &startY);

	saccadeSteps.push_front(position(startX, startY, targX, targY));
	int noTargetThreshold = 10;
	bool centred = target->targetCentred(&targX, &targY, colour);

	while(!centred &&
			failedDirectLinksCounter < 10 &&
			failedNeighbourCounter < 10 &&
			noTargetCounter < noTargetThreshold)
	{
		double curX, curY, x, y;
		bool success = head->getCurrentPosition(&curX, &curY);


		bool gotfield;
		PolarField* inputField = new PolarField();

		if(!(target->targetVisible()))
		{
			cout << colour << " target not visible" << endl;
			gotfield = false;
		}
		else
		{
			gotfield = isRetinaField(targX, targY);
			if(gotfield)
				inputField = getRetinaField(targX, targY);
			else
			{
				cout << "No input field available at this location" << endl;
			}

		}
		bool link = false;
		double relX, relY;

		PolarField* linkedOutput = new PolarField();
		if(gotfield)
		{
			if(inputField->isLearnt())
				gotfield = isLinkedOutput(inputField);
			else
				gotfield = false;
		}
		if(gotfield)
		{
			linkedOutput = getLinkedOutput(inputField);
			link = true;
			bool reachable = gotoField(linkedOutput);
			if(!reachable)
			{
				return false;
			}
		}
		else if(target->targetVisible())
		{
			float dist = 0;
			nearbyInput = getNearestLearntInput(targX, targY, &dist);	//ppm->getNearestLearntInputField((float)targX, (float)targY, &dist);
			double radius = inputField->getRadius();
			if(dist<(radius*1.5) && dist>0 && nearbyInput->getUsage()>=0  && nearestNeighbour)
			{
				cout << "Following nearby link" << endl;
				bool gotField = isLinkedOutput(nearbyInput);
				PolarField* nearbyOutput;
				if(gotField)
				{
					nearbyOutput = getLinkedOutput(nearbyInput);
					bool reachable = gotoField(nearbyOutput);
					if(!reachable)
					{
						return false;
					}
					followedNeighbour = true;
				}
			}
			else
			{
				followedNeighbour = false;
				head->babble();//makeRandomMove();	//blocks till movement complete
				head->getCurrentPosition(&x, &y);

				relX = x -curX;
				relY = y - curY;

			}
		}
		else
		{
			noTargetCounter ++;
			if(noTargetCounter >= noTargetThreshold)
			{
				return false;
			}

			cout << "making random movement" << endl;
			head->babble(); //makeRandomMove();	//blocks till movement complete
			head->getCurrentPosition(&x, &y);

			relX = x -curX;
			relY = y - curY;

		}
		while(!head->stationary())
		{
			Time::delay(0.3);
		}
		double endTargX, endTargY;
		double motorX, motorY;
		centred = target->targetCentred(&endTargX, &endTargY, colour);
		head->getCurrentPosition(&motorX, &motorY);
		saccadeSteps.push_front(position(motorX, motorY, endTargX, endTargY));


		/*
		 * Check the outcome of the movement performed.
		 * Use relative changes to update and add new links as appropriate
		 */
		if(link || followedNeighbour)
		{
			if(centred)
			{
				centred = target->targetCentred(&endTargX, &endTargY, colour);
				cout << "Successfully followed link to centre" << endl;
				if(!followedNeighbour)
				{
					PolarField* linkedOutput =  getLinkedOutput(inputField);

					if(learn)
					{
						linkedOutput->useField();
						inputField->useField();
						linkedOutput->setCalcdLinks(1);
						inputField->setCalcdLinks(1);

						FieldLink* link = head_ppm->getLink(inputField, linkedOutput);
						link->useField();
					}

				}
				else
				{
					PolarField* nearbyOutput = getLinkedOutput(nearbyInput);
					if(learn)
					{
						nearbyInput->useField();
						nearbyInput->setCalcdLinks(1);
						nearbyOutput->useField();
						nearbyOutput->setCalcdLinks(1);

						FieldLink* link = head_ppm->getLink(nearbyInput, nearbyOutput);
						link->useField();

						//PHS test extra
						makeLink(inputField, nearbyOutput);
						//end PHS test extra
					}
//					logEntry("Using successful neighbour to create a new link");

//					double endX, endY;
//					eye->getCurrentPosition(&endX, &endY);

//					relX = endX -curX;
//					relY = endY - curY;
					//Could use the fields here, but want to ensure we get the right values.
//					bool success = makeLink(startTargX, startTargY, endTargX, endTargY, relX, relY);

//					if(success) linksLearnt ++;
					followedNeighbour  = false;

				}
			}
			else
			{
				cout << "Followed link but it didn't take me to the centre" << endl;

				if((x>=head->xMax) || (x<=head->xMin) ||
						(y>=head->yMax) || (y<=head->yMin))
				{
					//target likely to be out of reach
					cout << "Target assumed to be out of reach" << endl;
					return false;
				}


				if(!followedNeighbour)
				{
					PolarField* output = getLinkedOutput(inputField);
					//ppm->deleteLink(inputField, output);
					if(learn)
					{
						inputField->setCalcdLinks(0);
						inputField->linkFailed();
						linkedOutput->setCalcdLinks(0);
						linkedOutput->linkFailed();

						FieldLink* link = head_ppm->getLink(inputField, linkedOutput);
						link->linkFailed();

						//PHS test extra
						cout << "Deleting a failed link" << endl;
						head_ppm->deleteLink(inputField, linkedOutput);
						//end PHS test extra
					}
					//update link
					if((targX==0 && targY==0) || (endTargX==0 && endTargY==0))
					{
					}
					else
					{
//						logEntry("updating learnt link");
						//Get actual motor encoders before assuming made full motor movement
//						double endX, endY;
//						eye->getCurrentPosition(&endX, &endY);
//
//						relX = endX - curX;
//						relY = endY - curY;

//						bool success = makeLink(targX, targY, endTargX, endTargY, relX, relY);
//						if(success) linksUpdated ++;
					}

					if(failedDirectLinksCounter >= 10)
					{
						cout << "Too many failed direct links, Target assumed to be out of reach" <<  endl;
						return false;
					}

					failedDirectLinksCounter ++;

				}
				else
				{
					PolarField* nearbyOutput = (PolarField*)getLinkedOutput(nearbyInput);
					if(!target->targetVisible() || failedNeighbourCounter>=3)
					{

						failedNeighbourCounter=0;
					}
					else
					{
						failedNeighbourCounter++;
					}
					if(learn)
					{
						FieldLink* link = head_ppm->getLink(nearbyInput, nearbyOutput);
						link->linkFailed();
						nearbyInput->linkFailed();
						nearbyOutput->linkFailed();
					}

					if((targX==0 && targY==0) || (endTargX==0 && endTargY==0))
					{
					}
					else
					{
//						logEntry("creating a calculated link from the neighbour link");
//
//						double endX, endY;
//						eye->getCurrentPosition(&endX, &endY);
//
//						relX = endX - curX;
//						relY = endY - curY;

//						bool success = makeLink(targX, targY, endTargX, endTargY, relX, relY);

//						if(success) linksLearnt ++;
					}
					followedNeighbour = false;

				}
			}
		}
		else
		{

			if((targX==0 && targY==0) || (endTargX==0 && endTargY==0))
			{
			}
//			else
//			{
////				bool success = makeLink(targX, targY, endTargX, endTargY, relX, relY);
//
////				if(success) linksLearnt ++;
//
//			}

		}

		while(!head->stationary())
		{
			Time::delay(0.3);
		}
		targX = endTargX;
		targY = endTargY;
	}//End of trying to fixate target

	centred = target->targetCentred(&targX, &targY, colour);

	//Either fixated target or given up trying
	if(centred)
	{


		if(!startInputField->isLearnt())
		{
			double endX, endY;
			head->getCurrentPosition(&endX, &endY);

			double relX, relY;
			relX = endX - startX;
			relY = endY - startY;

			if(!head_ppm->isNull(startInputField) &&
					!(startInputField->getXcoord()==0 && startInputField->getYcoord()==0))
			{

				PolarField* motor;
				bool gotfield = isMotorField(relX, relY);
				if(gotfield)
					motor = getMotorField(relX, relY);

				if(learn && gotfield)
				{
					bool ok = head_ppm->addLink(startInputField, motor);
					if(ok)
					{
						startInputField->linkAdded();
						motor->linkAdded();
					}
				}
			}

		}

		int linksLearnt;
		if(learn)
			linksLearnt = learnChain();


		cout << "learnt " << linksLearnt << " links from the saccade" << endl;
		return true;
	}
	else
	{
		return false;
	}

	return true;
}


int headSaccading::learnChain()	//returns the number of links learnt
{
	int linkCounter = 0;

	list<position>::iterator it;

	relativeMovement rel(saccadeSteps.front().eyeMotorX, saccadeSteps.front().eyeMotorY);

	int positionCounter = 1;
	for ( it=saccadeSteps.begin() ; it != saccadeSteps.end(); it++ )
	{
		position p = *it;
		rel -= p;
		if(p.targetX != 0 && p.targetY != 0)
		{
			//learn link that will take you direct to centre;
			bool success = makeLink(p.targetX, p.targetY, rel.relativeX, rel.relativeY);
			if(success)
				linkCounter ++;
		}
		positionCounter ++;
	}

	return linkCounter;
}

//Doesn't block
bool headSaccading::followLink(double relX, double relY)
{
	double x, y;
	head->getCurrentPosition(&x, &y);


	//Convert to actual motor values
	x += relX;
	y += relY;

	head->move(x,y, false);

	logEntry("Following a link with motor values provided by EyeHeadController");
}






/* *****************************************************************
 *
 * 		FIELD ACCESS METHODS
 *
 * *****************************************************************
 */

bool headSaccading::isRetinaField(double x, double y)
{
	logEntry("Requesting retina field at: ",x,y);
	PolarField* field = (PolarField*)head_ppm->getInputField((float)x, (float)y);
	if(head_ppm->isNull(field))
	{
		logEntry("Null input field returned");
		return false;
	}

	if(field->getXcoord() == 0 && field->getYcoord() == 0)
	{
		logEntry("0,0 input field returned");
		return false;
	}

	logEntry("Field returned: ");
	logField(field);
	return true;
}
PolarField* headSaccading::getRetinaField(double x, double y)
{
	return (PolarField*)head_ppm->getInputField((float)x, (float)y);
}

bool headSaccading::isMotorField(double x, double y)
{
	logEntry("Requesting motor field at: ",x,y);
	PolarField* field = (PolarField*)head_ppm->getOutputField((float)x, (float)y);
	if(head_ppm->isNull(field))
	{
		logEntry("Null output field returned");
		return false;
	}

	if(field->getXcoord() == 0 && field->getYcoord() == 0)
	{
		logEntry("0,0 output field returned");
		return false;
	}
	logEntry("Field returned: ");
	logField(field);
	return true;
}
PolarField* headSaccading::getMotorField(double x, double y)
{
	return (PolarField*)head_ppm->getOutputField((float)x, (float)y);
}


bool headSaccading::isLinkedInput(PolarField* motorField)
{
	logEntry("Requesting linked retina field for field:");
	logField(motorField);
	PolarField* retinaField = (PolarField*)head_ppm->getLinkedInput(motorField);
	if(head_ppm->isNull(retinaField))
	{
		logEntry("Null input field returned");
		return false;
	}

	else if(retinaField->getXcoord() == 0 && retinaField->getYcoord() == 0)
	{
		logEntry("0,0 output field returned");
		return false;
	}
	logEntry("Field returned: ");
	logField(retinaField);
	return true;
}
PolarField* headSaccading::getLinkedInput(PolarField* motorField)
{
	return (PolarField*)head_ppm->getLinkedInput(motorField);
}

bool headSaccading::isLinkedOutput(PolarField* retinaField)
{
	logEntry("Requesting linked motor field for field:");
	logField(retinaField);
	PolarField* motorField = (PolarField*)head_ppm->getLinkedOutput(retinaField);
	if(head_ppm->isNull(motorField))
	{
		logEntry("Null input field returned");
		return false;
	}

	else if(motorField->getXcoord() == 0 && motorField->getYcoord() == 0)
	{
		logEntry("0,0 output field returned");
		return false;
	}
	logEntry("Field returned: ");
	logField(motorField);
	return true;
}
PolarField* headSaccading::getLinkedOutput(PolarField* retinaField)
{
	return (PolarField*)head_ppm->getLinkedOutput(retinaField);
}



PolarField* headSaccading::getNearestLearntInput(double targX, double targY, float* dist)
{
	return head_ppm->getNearestLearntInputField((float)targX, (float)targY, dist);
}

PolarField* headSaccading::getNearestLearntOutput(double x, double y, float* dist)
{
	return head_ppm->getNearestLearntOutputField((float)x, (float)y, dist);
}

/*****************************************
 *  LOGGING FUNCTIONS:
 *****************************************
 */


void headSaccading::openLogs()
{
	string fullpath = path + "headmotorlog.txt";
	motorlogfile.open(fullpath.c_str());

	fullpath = path + "headlog.txt";
	logfile.open(fullpath.c_str());


	motorlogfile << "x y saccadeNo"<< endl;
}


void headSaccading::motorRecord(double relX, double relY, int saccadeCounter)
{
	motorlogfile << relX << " " << relY << " "
						<< saccadeCounter << " " << endl;
}

void headSaccading::logEntry(string message)
{
//	cout << message << endl;
	logfile << message << endl;
}

void headSaccading::logEntry(string message, double x, double y)
{
//	cout << message << x << ", " << y << endl;
	logfile << message << x << ", " << y << endl;
}

void headSaccading::logField(const Field* field)
{
//	cout << *field << endl;
	logfile << *field << endl;
}

void headSaccading::startSaccadeLog(int psaccadeCounter)
{
	saccadeCounter = psaccadeCounter;
	motorlogfile << "*******************************" << endl;
	motorlogfile << "Starting saccade: " << saccadeCounter << endl;

	logfile << "*******************************" << endl;
	logfile << "Starting saccade: " << saccadeCounter << endl;

}
void headSaccading::endSaccadeLog(bool success)
{

	if(success)
	{
		motorlogfile << "Fixated on saccade: " << saccadeCounter << endl;
		logfile << "Successfully fixated on saccade: " << saccadeCounter << endl;
	}
	else
	{
		motorlogfile << "Failed to fixate on saccade: " << saccadeCounter << endl;
		logfile << "Failed to fixate on saccade: " << saccadeCounter << endl;
	}
	motorlogfile << "*******************************" << endl;
	logfile << "*******************************" << endl;

}

void headSaccading::closeLogs()
{
	logfile << "**************END**************" << endl;
	logfile.close();

	motorlogfile << "**************END**************" << endl;
	motorlogfile.close();
}



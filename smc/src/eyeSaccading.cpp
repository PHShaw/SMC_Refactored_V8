/*
 * eyeSaccading.cpp
 *
 *  Created on: 6 Jun 2011
 *      Author: icub
 */

#include "eyeSaccading.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::dev;

eyeSaccading::eyeSaccading(eyeController* pEye, Target* pTarget, ffm* eye_ppm)
{
	eye = pEye;
	target = pTarget;
	ppm = eye_ppm;

	openLogs();
}

eyeSaccading::~eyeSaccading()
{
	closeLogs();
	saccadeSteps.clear();
}


///**
// * Blocks until the move is completed.
// */
//void eyeSaccading::makeRandomMove()
//{
//	eye->babble();
//
//}

bool eyeSaccading::gotoField(Field* motorfield)
{

	double relX = motorfield->getXcoord();		//relative motor values
	double relY = motorfield->getYcoord();
	string entry="Moving to an existing linked field at relative coordinates ";
	logEntry(entry, relX, relY);
	//convert to actual motor values (or figure out why relativeMove doesn't work)

	return followLink(relX, relY);


}


bool eyeSaccading::followLink(double relX, double relY)
{
	double x, y;
	bool success = eye->getCurrentPosition(&x, &y);
	if(!success)
	{
		logEntry("Failed to obtain current eye position");
//		eye->toRest();	//TODO: this may or may not be a good idea
		fr.failedToObtainedCurrentEyePosition = true;
		return false;
	}


	bool reachable = true;

	//Convert to actual motor values
	x += relX;
	y += relY;


	if((x>eye->xMax || x<eye->xMin) || (y>eye->yMax || y<eye->yMin))
	{
		reachable = false; //target likely to be out of reach
		logEntry("Go to field target is unreachable from here");
		fr.unreachable = true;
	}

	eye->move(x,y,true);	//safety checks and blocks till complete

	string entry = "Following a link with motor values ";
	logEntry(entry, x, y);
	motorlogfile << x << " " << y << " " << endl;
	return reachable;
}




bool eyeSaccading::makeLink(double startX, double startY, double endX, double endY, double motorRelX, double motorRelY, int calcdLinks)
{
	//get field for relative motor movement
	if((motorRelX > 60 || motorRelX < -60) || (motorRelY>53 || motorRelY<-53))
	{
		logEntry("Motor values outside valid range ", motorRelX, motorRelY);
		return false;
	}

//		-60.f, 60.f, -53.f, 53.f	Map motor ranges.

	PolarField* motor = new PolarField();
	bool gotfield = isMotorField(motorRelX, motorRelY);
	if(gotfield)
	{
		motor = getMotorField(motorRelX, motorRelY);
		bool ok = makeLink(startX, startY, endX, endY, motor, calcdLinks);
		logEntry("Adding link based on relative movement ", motorRelX, motorRelY);
		return ok;
	}
	else
		return false;

}

bool eyeSaccading::makeLink(double startX, double startY, double endX, double endY, PolarField* motor, int calcdLinks)
{
	double centreX = 320/2;
	double centreY = 240/2;
	double xOffset = endX - centreX;
	double yOffset = endY - centreY;

	double sourceX = startX - xOffset;
	double sourceY = startY - yOffset;

	if((sourceX > 320 || sourceX < 0) || (sourceY>240 || sourceY<0))
	{
		logEntry("Retina values outside valid range ", sourceX, sourceY);
		return false;
	}


	PolarField* source = new PolarField();
	bool gotfield = isRetinaField(sourceX, sourceY);// = getRetinaField(source, sourceX, sourceY);

	if(gotfield)
	{
		source = getRetinaField(sourceX, sourceY);
		bool ok = makeLink(source,motor, calcdLinks);
//		logfile << "Adding link based on relative movement" << endl;
		return ok;
	}
	else
		return false;
}

bool eyeSaccading:: makeLink(double sourceX, double sourceY, double motorRelX, double motorRelY, int calcdLinks)
{
	//get field for relative motor movement
	if((motorRelX > 60 || motorRelX < -60) || (motorRelY>53 || motorRelY<-53))
	{
		logEntry("Motor values outside valid range ", motorRelX, motorRelY);
		return false;
	}

	//check retina values
//	 0.f, 320.f, 0.f, 240.f
	if((sourceX > 320 || sourceX < 0) || (sourceY>240 || sourceY<0))
	{
		logEntry("Retina values outside valid range ", sourceX, sourceY);
		return false;
	}

	PolarField* motor = new PolarField();
	bool gotfield = isMotorField(motorRelX, motorRelY);

	if(gotfield)
	{
		motor = getMotorField(motorRelX, motorRelY);
		PolarField* source = new PolarField();
//		gotfield = getRetinaField(source, sourceX, sourceY);

		logEntry("Requesting retina field at: ",sourceX,sourceY);
		gotfield = isRetinaField(sourceX, sourceY);

		if(gotfield)
		{
			source = getRetinaField(sourceX, sourceY);
			bool ok = makeLink(source, motor, calcdLinks);
			string entry = "Adding link based on relative movement ";
			logEntry(entry, motorRelX, motorRelY);
			return ok;
		}
		else
			return false;

	}
	else
		return false;
}


bool eyeSaccading::makeLink(PolarField* source, PolarField* motor, int calcdLinks)
{
	if(ppm->containsLink(source, motor))
	{
		FieldLink* link = ppm->getLink(source, motor);
		link->useField();
//		source->useField();
		source->setCalcdLinks(calcdLinks);
//		motor->useField();
		motor->setCalcdLinks(calcdLinks);
		logEntry("strengthening existing link, rather than adding new link");
		return true;
	}
	else{
		bool ok = ppm->addLink(source, motor);
		if(ok)
		{
			source->linkAdded();
			motor->linkAdded();
			source->setCalcdLinks(calcdLinks);
			motor->setCalcdLinks(calcdLinks);
		}
		return ok;
	}

}

int eyeSaccading::learnChain()	//returns the number of links learnt
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
			bool success = makeLink(p.targetX, p.targetY, rel.relativeX, rel.relativeY, positionCounter);
			if(success)
			{
				linkCounter ++;
				eyeLinkLog << p.targetX << " " << p.targetY << " " << rel.relativeX << " " << rel.relativeY << " ";
				time_t current = time(NULL);
				eyeLinkLog << current << endl;
			}
		}
		positionCounter ++;
	}

	return linkCounter;
}

/**
 * For use once a reasonable saccade map has been learnt
 * A single movement will be made, but there is no guarantee it will reach the target
 * No consideration is given for the quality of the link used, and only the local link is considered.
 * No neighbouring links are used, so if the current location is not covered this will fail.
 * Link usage is updated depending on the success or failure of the link.
 */
bool eyeSaccading::simpleSaccade(double targX, double targY, string colour, bool check)
{
	fr.reset();
	double x,y;
	eye->getCurrentPosition(&x, &y);
	fr.startX = x;
	fr.startY = y;

	double dist;
	if(target->fovea(targX, targY, &dist))
		return true;

	float distf;
	PolarField* startInputField = new PolarField();
	if(!isRetinaField(targX, targY))
	{
		cout <<"Failed to obtain input field for simple saccade" << endl;
		startInputField = getNearestLearntInput(targX, targY, &distf);
		fr.nearestInput = startInputField;
		fr.dist = distf;
		fr.nearestNeighbour = true;
	}
	else
		startInputField = getRetinaField(targX, targY);
	fr.startInput = startInputField;

	PolarField* linkedOutput = new PolarField();
	bool gotField = isLinkedOutput(startInputField);
	if(gotField)
	{
		linkedOutput = getLinkedOutput(startInputField);
	}
	else
	{
		cout << "Input field not directly linked for simple saccade" << endl;
		PolarField* nearestInput = getNearestLearntInput(targX, targY, &distf);
		linkedOutput = getLinkedOutput(nearestInput);
		fr.nearestNeighbour = true;
		fr.dist = distf;
		fr.nearestInput = nearestInput;
	}
	fr.output = linkedOutput;

	if(!gotoField(linkedOutput))
	{
		cout << "Target is not reachable for simple saccade" << endl;
		if(!check)
		{
			eye->getCurrentPosition(&x, &y);
			fr.endX = x;
			fr.endY = y;
			return false;
		}
	}

	if(check)
	{
		if(target->targetCentred(&targX, &targY, colour))
		{
			autoCenter(targX, targY, colour);
	//		centred = target->targetCentred(&targX, &targY, colour);
			startInputField->useField();
			linkedOutput->useField();

			FieldLink* link = ppm->getLink(startInputField, linkedOutput);
			if(link==0)
			{
				cout << "Adding a new link" << endl;
				makeLink(startInputField, linkedOutput, 1);
			}
			else
				link->useField();
			return true;
		}
		else
		{
			autoCenter(targX, targY, colour);
	//		centred = target->targetCentred(&targX, &targY, colour);
	//		startInputField->linkFailed();		//Note: if tracking a moving object, the it is likely to have moved
	//		linkedOutput->linkFailed();

	//		FieldLink* link = ppm->getLink(startInputField, linkedOutput);
	//		if(link!=0)
	//			link->linkFailed();
			return false;
		}
	}
	else
	{
		return true;	//can't check vision, but a movement was made
	}
}

/**
 * Throws segmentation fault whilst trying to delete a string for some reason.
 */
bool eyeSaccading::saccade(int pSaccadeCounter, double targX, double targY)
{
	bool success = saccade(pSaccadeCounter, targX, targY, target->getColour());
	return success;
}



bool eyeSaccading::saccade(int pSaccadeCounter, double targX, double targY, string colour)
{
	saccadeCounter = pSaccadeCounter;
	resetStats();
	startSaccadeLog();

	saccadeSteps.clear();

	int failedDirectLinksCounter = 0;
	int failedNeighbourCounter = 0;
	bool followedNeighbour = false;
	int noTargetCounter=0;
	bool randomMove = false;


	PolarField* startInputField = new PolarField();
	if(!isRetinaField(targX, targY))
	{
		logEntry("Failed to obtain start input field");
		endSaccadeLog(false);
		return false;
	}
	else
		startInputField = getRetinaField(targX, targY);
	PolarField* nearbyInput = new PolarField();

	logEntry("Retrieved input field for target ", targX, targY);
	logField(startInputField);


	while(!eye->stationary())
	{
		Time::delay(0.2);
	}

	double startTargX = targX, startTargY = targY;
	string startColour = colour;

	double startX, startY;
	bool success = eye->getCurrentPosition(&startX, &startY);
	if(!success)
	{
		endSaccadeLog(false);
		return false;
	}

	saccadeSteps.push_front(position(startX, startY, targX, targY));
	string entry = "Starting saccade ";
	entry = appendInt(entry, saccadeCounter);
	entry += " from ";
	logEntry(entry, startX, startY);

	int noTargetThreshold = 100;
	noTargetThreshold = max((int)(100.0/(saccadeCounter+1)),noTargetThreshold);

	bool centred = target->targetCentred(&targX, &targY, colour);

	while(!centred &&
			failedDirectLinksCounter < 10 &&
			failedNeighbourCounter < 10 &&
			noTargetCounter < noTargetThreshold)
	{
		double curX, curY, x, y;
		bool success = eye->getCurrentPosition(&curX, &curY);
		if(!success)
		{
			cout << "Current position unknown" << endl;
			continue;
		}


		//stats
		if(randomMove && penultimateMoveWasLink)
		{
			penultimateMoveWasLink = false;
		}
		randomMove = false;
		//end stats


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

		/*
		 * Look to see if the input field is currently linked to a motor field.
		 * If so, follow the link, otherwise look for a nearly learnt field,
		 * follow if available or make a random movement.
		 */

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
			logEntry("Input field is linked to field ");
			logField(linkedOutput);
			link = true;
			logEntry("following direct link");
			motorRecord(linkedOutput->getXcoord(), linkedOutput->getYcoord());
			bool reachable = gotoField(linkedOutput);
			if(!reachable)
			{
				logEntry("Target unreachable from here");
				endSaccadeLog(false);
				return false;
			}
		}

		else if(target->targetVisible())
		{
			float dist = 0;
			nearbyInput = getNearestLearntInput(targX, targY, &dist);	//ppm->getNearestLearntInputField((float)targX, (float)targY, &dist);
			double radius = inputField->getRadius();
			if(dist<(radius*1.5) && dist>0 && nearbyInput->getUsage()>=0  && NEAREST_NEIGHBOUR)
			{
				cout << "Usage of nearby input: " << nearbyInput->getUsage() << endl;
				bool gotField = isLinkedOutput(nearbyInput);
				PolarField* nearbyOutput;
				if(gotField)
				{
					nearbyOutput = getLinkedOutput(nearbyInput);
					string entry = "Found a nearby learnt field at distance ";
					entry = appendInt(entry, dist);
					logEntry(entry);
					logEntry("Nearby field is: ");
					logField(nearbyOutput);
					logEntry("following nearest neighbour");
					motorRecord(nearbyOutput->getXcoord(), nearbyOutput->getYcoord());
					bool reachable = gotoField(nearbyOutput);
					if(!reachable)
					{
						endSaccadeLog(false);
						return false;
					}
					followedNeighbour = true;
				}


			}
			else
			{
				if(!NEAREST_NEIGHBOUR)
				{
					logEntry("not considering nearest neighbours");
				}
				else if(dist>=10000)
				{
					logEntry("nearest neighbour was too far away, so making random movement instead");
				}
				else
				{
					logEntry("nearest neighbour was unreliable, so making random movement instead");
				}

				followedNeighbour = false;
				eye->babble();//makeRandomMove();	//blocks till movement complete
				eye->getCurrentPosition(&x, &y);

				relX = x -curX;
				relY = y - curY;
				motorRecord(relX, relY);

				logEntry("Made a random movement to ", x, y);
			}
		}
		else
		{
			noTargetCounter ++;
			if(noTargetCounter >= noTargetThreshold)
			{
				cout << "Not likely to be able to reach an invisible target" << endl;
				cout << "Or started on a colour that I've not been able to locate again" << endl;
				endSaccadeLog(false);
				return false;
			}

			cout << "making random movement" << endl;
			eye->babble(); //makeRandomMove();	//blocks till movement complete
			eye->getCurrentPosition(&x, &y);

			relX = x -curX;
			relY = y - curY;
			motorRecord(relX, relY);

			logEntry("Made a random movement to ", x, y);
		}
		while(!eye->stationary())
		{
			Time::delay(0.3);
		}
		double endTargX, endTargY;
		double motorX, motorY;
		centred = target->targetCentred(&endTargX, &endTargY, colour);
		eye->getCurrentPosition(&motorX, &motorY);
		saccadeSteps.push_front(position(motorX, motorY, endTargX, endTargY));

		/*
		 * Check the outcome of the movement performed.
		 * Use relative changes to update and add new links as appropriate
		 */
		if(link || followedNeighbour)
		{
			if(centred)
			{
				autoCenter(endTargX, endTargY, colour);
				centred = target->targetCentred(&endTargX, &endTargY, colour);
				logEntry("Successfully followed link to centre");
				if(!followedNeighbour)
				{
					PolarField* linkedOutput = (PolarField*)ppm->getLinkedOutput(inputField);

					if(params.LEARN)
					{
						linkedOutput->useField();
						inputField->useField();
						linkedOutput->setCalcdLinks(1);
						inputField->setCalcdLinks(1);

						FieldLink* link = ppm->getLink(inputField, linkedOutput);
						link->useField();
					}

					//stats
					successfulDirectLink = true;
				}
				else
				{
					PolarField* nearbyOutput = (PolarField*)ppm->getLinkedOutput(nearbyInput);
					if(params.LEARN)
					{
						nearbyInput->useField();
						nearbyInput->setCalcdLinks(1);
						nearbyOutput->useField();
						nearbyOutput->setCalcdLinks(1);

						FieldLink* link = ppm->getLink(nearbyInput, nearbyOutput);
						link->useField();

						//PHS test extra
						makeLink(inputField, nearbyOutput, 1);
						//end PHS test extra
					}
//					logEntry("Using successful neighbour to create a new link");

//					double endX, endY;
//					eye->getCurrentPosition(&endX, &endY);

//					relX = endX -curX;
//					relY = endY - curY;
					//Could use the fields here, but want to ensure we get the right values.
//					bool success = makeLink(startTargX, startTargY, endTargX, endTargY, relX, relY);
					possibleLinkstoLearn ++;
//					if(success) linksLearnt ++;
					followedNeighbour  = false;

					//stats
					neighbourCounter ++;
					successfulNeighbourLink = true;
				}
			}
			else
			{
				logEntry("Followed link but it didn't take me to the centre");

				if((x>=29) || (x<=-29) ||
						(y>=18) || (y<=-35))
				{
					//target likely to be out of reach
					cout << "Target assumed to be out of reach" << endl;
					endSaccadeLog(false);
					return false;
				}


				if(!followedNeighbour)
				{


					PolarField* output = (PolarField*)ppm->getLinkedOutput(inputField);
					//ppm->deleteLink(inputField, output);
					if(params.LEARN)
					{
						inputField->setCalcdLinks(0);
						inputField->linkFailed();
						linkedOutput->setCalcdLinks(0);
						linkedOutput->linkFailed();

						FieldLink* link = ppm->getLink(inputField, linkedOutput);
						link->linkFailed();

						//PHS test extra
						cout << "Deleting a failed link" << endl;
						ppm->deleteLink(inputField, linkedOutput);
						//end PHS test extra
					}
					//update link
					if((targX==0 && targY==0) || (endTargX==0 && endTargY==0))
					{
						logEntry("One of the targets was not visible to update the link");
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
						endSaccadeLog(false);
						return false;
					}

					failedDirectLinksCounter ++;

					//stats
					unsuccesfulDirectLinkCounter ++;
					penultimateMoveWasLink = true;
				}
				else
				{
					PolarField* nearbyOutput = (PolarField*)ppm->getLinkedOutput(nearbyInput);
					if(!target->targetVisible() || failedNeighbourCounter>=3)
					{

						failedNeighbourCounter=0;
					}
					else
					{
						failedNeighbourCounter++;
					}
					logEntry("Dis-using a failed neighbour");
					if(params.LEARN)
					{
						FieldLink* link = ppm->getLink(nearbyInput, nearbyOutput);
						link->linkFailed();
						nearbyInput->linkFailed();
						nearbyOutput->linkFailed();
					}

					possibleLinkstoLearn ++;
					if((targX==0 && targY==0) || (endTargX==0 && endTargY==0))
					{
						logEntry("One of the targets was not visible to create link from the neighbour");
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

					//stats
					neighbourCounter ++;
					penultimateMoveWasLink = true;
				}
			}
		}
		else
		{

			possibleLinkstoLearn ++;
			if((targX==0 && targY==0) || (endTargX==0 && endTargY==0))
			{
				logEntry("One of the targets was not visible to learn a link");
			}
//			else
//			{
////				bool success = makeLink(targX, targY, endTargX, endTargY, relX, relY);
//
////				if(success) linksLearnt ++;
//
//			}

			//stats
			randomMove = true;
		}

		stepCounter ++;
		while(!eye->stationary())
		{
			Time::delay(0.3);
		}
		targX = endTargX;
		targY = endTargY;
	}//End of trying to fixate target



	autoCenter(targX, targY, colour);
	centred = target->targetCentred(&targX, &targY, colour);

	//Either fixated target or given up trying
	if(centred)
	{


		if(!startInputField->isLearnt())
		{
			double endX, endY;
			eye->getCurrentPosition(&endX, &endY);

			double relX, relY;
			relX = endX - startX;
			relY = endY - startY;

			possibleLinkstoLearn ++;
			if(!ppm->isNull(startInputField) &&
					!(startInputField->getXcoord()==0 && startInputField->getYcoord()==0))
			{

				PolarField* motor;
				bool gotfield = isMotorField(relX, relY);
				if(gotfield)
					motor = getMotorField(relX, relY);

				if(params.LEARN && gotfield)
				{
					bool ok = ppm->addLink(startInputField, motor);
					if(ok)
					{
						startInputField->linkAdded();
						motor->linkAdded();
					}
					linksLearnt ++;
					startInputField->setCalcdLinks(stepCounter);
					motor->setCalcdLinks(stepCounter);
				}
			}

		}


		if(params.LEARN)
			linksLearnt = learnChain();


		entry = "fixated on saccade ";
		entry = appendInt(entry, saccadeCounter);
		entry += " after ";
		entry = appendInt(entry, stepCounter);
		entry += " steps";

		logEntry(entry);
		cout << "learnt " << linksLearnt << " links from the saccade" << endl;
		endSaccadeLog(true);
		return true;
	}
	else
	{
		logEntry("Unable to locate target");
		endSaccadeLog(false);
		return false;
	}
	endSaccadeLog(true);
	return true;

}





/* *****************************************************************
 *
 * 		FIELD ACCESS METHODS
 *
 * *****************************************************************
 */

/**
 * NOTE: This methods return copies of the fields, not the fields themselves
 *			Therefore, changes to these fields will not be reflected in later requests.
 */
//bool eyeSaccading::getRetinaField(PolarField* field, double x, double y)
//{
//	logEntry("Requesting retina field at: ",x,y);
//	PolarField* temp = (PolarField*)ppm->getInputField((float)x, (float)y);
//	if(ppm->isNull(temp))
//	{
//		logEntry("Null input field returned");
//		return false;
//	}
//
//	if(temp->getXcoord() == 0 && temp->getYcoord() == 0)
//	{
//		logEntry("0,0 input field returned");
//		return false;
//	}
//
//	*field = *temp;
//
//	logEntry("Field returned: ");
//	logField(field);
//	return true;
//}

bool eyeSaccading::isRetinaField(double x, double y)
{
	logEntry("Requesting retina field at: ",x,y);
	PolarField* temp = (PolarField*)ppm->getInputField((float)x, (float)y);
	if(ppm->isNull(temp))
	{
		logEntry("Null input field returned");
		return false;
	}

	if(temp->getXcoord() == 0 && temp->getYcoord() == 0)
	{
		logEntry("0,0 input field returned");
		return false;
	}
	logEntry("Field returned: ");
	logField(temp);
	return true;
}

/*
 * NOTE: This method does not check if the field actually exists, use isRetinaField(x,y) for that!
 */
PolarField* eyeSaccading::getRetinaField(double x, double y)
{
	return (PolarField*)ppm->getInputField((float)x, (float)y);
}


bool eyeSaccading::isMotorField(double x, double y)
{
	logEntry("Requesting motor field at: ",x,y);
	PolarField* temp = (PolarField*)ppm->getOutputField((float)x, (float)y);
	if(ppm->isNull(temp))
	{
		logEntry("Null output field returned");
		return false;
	}

	if(temp->getXcoord() == 0 && temp->getYcoord() == 0)
	{
		logEntry("0,0 output field returned");
		return false;
	}
//	*field = *temp;
	logEntry("Field returned: ");
	logField(temp);
	return true;
}

PolarField* eyeSaccading::getMotorField(double x, double y)
{
	return (PolarField*)ppm->getOutputField((float)x, (float)y);
}


bool eyeSaccading::isLinkedInput(PolarField* motorField)
{
	logEntry("Requesting linked retina field for field:");
	logField(motorField);

	PolarField* temp = (PolarField*)ppm->getLinkedInput(motorField);
	if(ppm->isNull(temp))
	{
		logEntry("Null input field returned");
		return false;
	}

	else if(temp->getXcoord() == 0 && temp->getYcoord() == 0)
	{
		logEntry("0,0 output field returned");
		return false;
	}
//	*retinaField = *temp;
	logEntry("Field returned: ");
	logField(temp);
	return true;
}

PolarField* eyeSaccading::getLinkedInput(PolarField* motorField)
{
	return (PolarField*)ppm->getLinkedInput(motorField);
}

bool eyeSaccading::isLinkedOutput(PolarField* retinaField)
{
	logEntry("Requesting linked motor field for field:");
	logField(retinaField);
	PolarField* temp = (PolarField*)ppm->getLinkedOutput(retinaField);
	if(ppm->isNull(temp))
	{
		logEntry("Null input field returned");
		return false;
	}

	else if(temp->getXcoord() == 0 && temp->getYcoord() == 0)
	{
		logEntry("0,0 output field returned");
		return false;
	}

	double x, y;
	x = temp->getXcoord();
	y = temp->getYcoord();


	if((x > 60 || x < -60) || (y>53 || y<-53))
	{
		logEntry("Obtained a linked field, but its motor values were outside valid range ", x, y);
		bool success = ppm->deleteLink(retinaField, temp);
		if(success)
		{
			temp->setCalcdLinks(0);
			if(isLinkedOutput(retinaField))
			{
//				logEntry("Field returned: ");
//				logField(temp);
//				*motorField = *temp;
				return true;
			}
			else
			{
				retinaField->setCalcdLinks(0);
				return false;
			}
		}
		return false;
	}

//	*motorField = *temp;
	logEntry("Field returned: ");
	logField(temp);
	return true;
}

PolarField* eyeSaccading::getLinkedOutput(PolarField* retinaField)
{
	return (PolarField*)ppm->getLinkedOutput(retinaField);
}


PolarField* eyeSaccading::getNearestLearntInput(double targX, double targY, float* dist)
{
	return ppm->getNearestLearntInputField((float)targX, (float)targY, dist);
}

PolarField* eyeSaccading::getNearestLearntOutput(double x, double y, float* dist)
{
	return ppm->getNearestLearntOutputField((float)x, (float)y, dist);
}

PolarField* eyeSaccading::getNearestReliableLearntInput(double targX, double targY, float* dist)
{
	return ppm->getNearestReliableLearntInputField((float)targX, (float)targY, dist);
}

PolarField* eyeSaccading::getNearestReliableLearntOutput(double x, double y, float* dist)
{
	return ppm->getNearestReliableLearntOutputField((float)x, (float)y, dist);
}


FieldLink* eyeSaccading::getLinkFromOutput(PolarField* motorField)
{
	Field* input = ppm->getLinkedInput(motorField);
	return ppm->getLink(input, motorField);
}


bool eyeSaccading::getDepth(string colour, double* depth)
{
	double rTargX, rTargY, lTargX, lTargY, xDiff;
	target->getTarget(&rTargX, &rTargY, colour);
	target->getLeft(&lTargX, &lTargY, colour);

	if(rTargX==0 && rTargY==0)
	{
		Time::delay(0.1);
		target->getLastTargs(&rTargX, &rTargY);
		if(rTargX==0 && rTargY==0)
			return false;	//target not visible in one of the eyes
	}
	if( lTargX==0 && lTargY==0)
	{
		Time::delay(0.1);
		target->getLeft(&lTargX, &lTargY, colour);
		if(lTargX==0 && lTargY==0)
			return false;	//target not visible in one of the eyes
	}

	xDiff = rTargX - lTargX;
//	printf("Calculating depth for xDiff: %.2f\n",xDiff);

	if(xDiff<1)	//equations generated in matlab from data recorded with eyes fixated on targets at diff depths
	{
//		printf("Using eqn 1\n");
		*depth = 0.00011*pow(xDiff,3) + 0.015*pow(xDiff,2) + 0.91*xDiff + 35;
	}
	else
	{
//		printf("Using eqn 2\n");
		*depth = 0.025*pow(xDiff,3) - 0.79*pow(xDiff,2) + 9.9*xDiff + 21;
	}


//	*depth = -6.5e-11*pow(xDiff,7) + 9.2e-08*pow(xDiff,6) + 7.2e-06*pow(xDiff,5) + 8.1e-05*pow(xDiff,4) - 0.0034*pow(xDiff,3) - 0.022*pow(xDiff,2) + 1.8*xDiff + 52;
	logfile << rTargX << " " << rTargY << " " << lTargX << " " << lTargY << " " << xDiff << " " << *depth << endl;


//	double rTargX, rTargY, lTargX, lTargY;
//	double panR, panL, tilt;	// A, B
//	double vergence;
//
//	target->getTarget(&rTargX, &rTargY, colour);
//	if(target->targetCentred())
//	{
//		eye->getCurrentPosition(&panR,&tilt);
//	}
//	else
//	{
//		if(rTargX==0 && rTargY==0)
//			return false;
//
//		if(isRetinaField(rTargX, rTargY))
//		{
//			PolarField* input = getRetinaField(rTargX, rTargY);
//			if(isLinkedOutput(input))
//			{
//				PolarField* output = getLinkedOutput(input);
//				panR = 90 - output->getXcoord();
//				tilt = output->getYcoord();
//			}
//			else
//				return false;
//		}
//		else
//			return false;
//	}
//
//
//	target->getLeft(&lTargX, &lTargY, colour);
//	if(lTargX==0 && lTargY==0)
//			return false;
//	if(isRetinaField(lTargX, lTargY))
//	{
//		PolarField* input = getRetinaField(lTargX, lTargY);
//		if(isLinkedOutput(input))
//		{
//			PolarField* output = getLinkedOutput(input);
//			panL = 90 + output->getXcoord();
//			tilt += output->getYcoord();
//			tilt /=2;
//		}
//		else
//			return false;
//	}
//	else
//		return false;
//
//	vergence = 180 - panL - panR;	// C
//
//	// a/sinA=b/sinB=c/sinC
//	// b = distance between eyes / 2 = 35mm
//	double side_c = 70;	// c' = c/2;
//
//	double side_b =side_c*sin(toRadians(panL))/sin(toRadians(vergence));
//
////	cout << "rTargX: "<< rTargX << ", rTargY: " << rTargY << endl;
////	cout << "lTargX: "<< lTargX << ", lTargY: " << lTargY << endl;
////	cout << "panR: " << panR << ", panL: " << panL << ", tilt: " << tilt << endl;
////
////	cout << "vergence: " << vergence << endl;
//
//	if(vergence==0)
//		return false;
//	*depth = sqrt(pow(side_b,2)+pow(side_c/2,2)-(2*side_b*(side_c/2)*cos(toRadians(panL))));
//
//	logfile << rTargX << " " << rTargY << " " << lTargX << " " << lTargY << " " << panR << " "
//			<< panL << " " << tilt << " " << vergence << " " << *depth << endl;
	return true;


}


bool eyeSaccading::autoCenter(double targX, double targY, string colour, bool check)
{
	printf("attempting to auto centre\n");

	int centreX = 320/2;
	int centreY = 240/2;
	int foveaRadius = 16;

	double xDeg = 9.5/2;	//diameter
	double yDeg = 9.0/2;

	int counter = 0;
	double dist;
	bool success = true;
//	target->fovea(targX,targY, &dist);
	if(targX==0 && targY == 0)
		return false;
//	while(dist>1 && counter < 3 && success)
//	{

		double xOffset = targX - centreX;
		double yOffset = targY - centreY;
	//	printf("proportion (x,y) (%.2f,%.2f)\n", xOffset, yOffset);

		double xProportion = xOffset/foveaRadius;
		double yProportion = yOffset/foveaRadius;
	//	printf("proportion (x,y) (%.2f,%.2f)\n", xProportion, yProportion);

		double xMovement = xDeg * xProportion;
		double yMovement = yDeg * yProportion;
		printf("movement (x,y) (%.2f,%.2f)\n", xMovement, yMovement);

		double curX, curY;



		success = eye->getCurrentPosition(&curX, &curY);
		eye->move(curX + xMovement, curY-yMovement, true);
		if(check)
		{
			Time::delay(0.2);
			target->getTarget(&targX, &targY, colour);
			if(!target->targetVisible())
				return false;
			target->fovea(targX, targY, &dist);
			counter ++;
		}
		else
		{
			return true;
		}
//	}

	if(dist<2)
		success = true;
	else
		success = false;
	return success;
}


bool eyeSaccading::verge(double lTargX, double lTargY, double rTargX, double rTargY)
{
	double difference = rTargX - lTargX;

	if(difference>-0.5 && difference <0.5)
	{
		cout << "Target is in focus" << endl;
		return true;
	}
	else if(difference>0)
	{ //If eyes are already verged, can diverge the eyes to fixate, without resetting each time...
		cout << "Target is divergent or mis-matched: r-l="<< difference <<endl;
		double current;
		double adjustment = -2;
		eye->getVergence(&current);
		if(current > 1)
		{
			autoCenter(rTargX, rTargY, "", false);		//Keep right eye centred on the target, move left eye out to it

			if (difference < 1)
				adjustment = -0.5;
			else if (difference < 2.5)
				adjustment = -0.75;
			else if (difference < 4.5)
				adjustment = -1;

			eye->getVergence(&current);
			eye->verg(current+adjustment,true);
		}
	}
	else
	{
		//go cross eyed on target
		//Meaningful difference is -1 --> -6, beyond this, it is likely that the object is not recognised equally in both eyes
		cout << "Starting to focus on target: r-l="<<difference << endl;

		double current;
		double adjustment = 1.5;
		autoCenter(rTargX, rTargY, "", false);		//Keep right eye centred on the target, bring left eye in to it

		adjustment = 2;
		if (difference > -1)
			adjustment = 0.5;
		else if (difference > -2.5)
			adjustment = 0.75;
		else if (difference > -4.5)
			adjustment = 1;

		eye->getVergence(&current);
		eye->verg(current+adjustment,true);
	}

	return false;	//Made a movement to verge on target, but need to call again to see if it is now in focus.
}


bool eyeSaccading::verge(string colour)
{
	//Assuming right eye is currently centred on the target
	double lTargX, lTargY;
	double rTargX, rTargY;

	bool success = target->getTarget(&rTargX, &rTargY, colour);
	success &= target->getLeft(&lTargX, &lTargY, colour);

	if(!success)
	{
		cout << "Failed to located target in both images for vergence" << endl;
		return success;
	}


	double difference = rTargX - lTargX;

	double newDiff = difference;
	double current;
	double adjustment;
	eye->getVergence(&current);
	int counter = 0;
	do
	{
		if(newDiff>-0.6 && newDiff <0.6)
		{
			cout << "Target is in focus" << endl;
			return true;
		}
		else if(current > 1 && newDiff>0)
		{
			adjustment = -3;
			if (newDiff < 1)
				adjustment = -0.25;
			else if (newDiff < 1.5)
				adjustment = -0.5;
			else if (newDiff < 2.5)
				adjustment = -0.75;
			else if (newDiff < 3.25)
				adjustment = -1;
			else if (newDiff < 4.5)
				adjustment = -1.5;
			else if (newDiff < 7)
				adjustment = -2;
		}
		else if(current < 1 && newDiff>0)
		{
			cout << "Target is divergent or mis-matched: r-l="<< newDiff <<endl;
			return false;
		}
		else
		{
			adjustment = 3;
			if (newDiff > -1)
				adjustment = 0.25;
			else if (newDiff > -1.5)
				adjustment = 0.5;
			else if (newDiff > -2.5)
				adjustment = 0.75;
			else if (newDiff > -3.24)
				adjustment = 1;
			else if (newDiff > -4.5)
				adjustment = 1.5;
			else if (newDiff > -7)
				adjustment = 2;
		}

		eye->getVergence(&current);
		eye->verg(current+adjustment,true);
		counter ++;
		difference = newDiff;

		yarp::os::Time::delay(0.15);
		target->getTarget(&rTargX, &rTargY, colour);
		target->getLeft(&lTargX, &lTargY, colour);
		newDiff = rTargX - lTargX;
		cout << "Target difference: r-l="<<newDiff << " vergence: " << (current+adjustment) << endl;
		autoCenter(rTargX, rTargY, colour, false);		//Keep right eye centred on the target, move left eye out to it


		if(counter>10)
			break;

	}while(newDiff > 0.5 || newDiff < -0.5);

	return true;

//	 if(difference>0)
//	{ //If eyes are already verged, can diverge the eyes to fixate, without resetting each time...
//		cout << "Target is divergent or mis-matched: r-l="<< difference <<endl;
//		double newDiff = difference;
//		double current;
//		double adjustment = 1.5;
//		eye->getVergence(&current);
//
//
//
//		if(current > 1)
//		{
//			do
//			{
//				yarp::os::Time::delay(0.15);
//				target->getTarget(&rTargX, &rTargY, colour);
//				autoCenter(rTargX, rTargY, colour);		//Keep right eye centred on the target, move left eye out to it
//
//				difference = newDiff;
//				adjustment = -2;
//				if (newDiff < 1)
//					adjustment = -0.5;
//				else if (newDiff < 2.5)
//					adjustment = -0.75;
//				else if (newDiff < 4.5)
//					adjustment = -1;
//
//				eye->getVergence(&current);
//				eye->verg(current+adjustment,true);
//				yarp::os::Time::delay(0.15);
//
//				target->getTarget(&rTargX, &rTargY, colour);
//				target->getLeft(&lTargX, &lTargY, colour);
//				newDiff = rTargX - lTargX;
//
//				cout << "Target difference: r-l="<<newDiff << " vergence: " << (current+adjustment) << endl;
//			}while(newDiff > 0.5);
//			if(newDiff < -1)
//			{
//				eye->getVergence(&current);
//				eye->verg(current-(adjustment/2),true);
//			}
//			return true;
//		}
//		else
//			return false;
//	}
//	else
//	{
//		//go cross eyed on target
//		//Meaningful difference is -1 --> -6, beyond this, it is likely that the object is not recognised equally in both eyes
//		cout << "Starting to focus on target: r-l="<<difference << endl;
//
//		double newDiff = difference;
//		double current;
//		double adjustment = 1.5;
//		do
//		{
//			yarp::os::Time::delay(0.2);
//			target->getTarget(&rTargX, &rTargY, colour);
//			autoCenter(rTargX, rTargY, colour);		//Keep right eye centred on the target, bring left eye in to it
//
//			difference = newDiff;
//			adjustment = 2;
//			if (newDiff > -1)
//				adjustment = 0.5;
//			else if (newDiff > -2.5)
//				adjustment = 0.75;
//			else if (newDiff > -4.5)
//				adjustment = 1;
//
//			eye->getVergence(&current);
//			eye->verg(current+adjustment,true);
//			yarp::os::Time::delay(0.2);
//
//			target->getTarget(&rTargX, &rTargY, colour);
//			target->getLeft(&lTargX, &lTargY, colour);
//			newDiff = rTargX - lTargX;
//
//			cout << "Target difference: r-l="<<newDiff << " vergence: " << (current+adjustment) << endl;
//
//		}while(newDiff < -0.5);
//
//		if(newDiff > 1)
//		{
//			eye->getVergence(&current);
//			eye->verg(current-(adjustment/2),true);
//		}
//		return true;
//	}

}



/* *****************************************************************
 *
 * 		LOGGING METHODS
 *
 * *****************************************************************
 */

void eyeSaccading::openLogs()
{
	string fullpath = params.m_PATH + "eyemotorlog.txt";


	motorlogfile.open(fullpath.c_str());
	motorlogfile << "x y saccadeNo stepCount" << endl;

	fullpath = params.m_PATH + "eyelog.txt";

	logfile.open(fullpath.c_str());

	fullpath = params.m_PATH + "eyeLinkLog.txt";
	eyeLinkLog.open(fullpath.c_str());
	eyeLinkLog << "retX retY motX motY time" << endl;
}


void eyeSaccading::motorRecord(double relX, double relY)
{
	motorlogfile << relX << " " << relY << " "
						<< saccadeCounter << " " << stepCounter << endl;
}

void eyeSaccading::logEntry(string message)
{
//	cout << message << endl;
	logfile << message << endl;
}

void eyeSaccading::logEntry(string message,double x,double y)
{

//	cout << message << x << ", " << y << endl;
	logfile << message << x << ", " << y << endl;
}


void eyeSaccading::logField(const Field* field)
{
//	cout << *field << endl;
	logfile << *field << endl;
}

void eyeSaccading::startSaccadeLog()
{
	motorlogfile << "*******************************" << endl;
	motorlogfile << "Starting saccade: " << saccadeCounter << endl;

	logfile << "*******************************" << endl;
	logfile << "Starting saccade: " << saccadeCounter << endl;
}

void eyeSaccading::endSaccadeLog(bool success)
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

void eyeSaccading::closeLogs()
{
	logfile << "**************END**************" << endl;
	logfile.close();

	motorlogfile << "**************END**************" << endl;
	motorlogfile.close();

	eyeLinkLog << "**************END**************" << endl;
	eyeLinkLog.close();
}

void eyeSaccading::stopLearning()
{
	params.LEARN=false;
}

void eyeSaccading::startLearning()
{
	params.LEARN=true;
}

/* *****************************************************************
 *
 * 		STATISTICS METHODS
 *
 * *****************************************************************
 */


void eyeSaccading::resetStats()
{
	//stats reset
	stepCounter = 0;
	unsuccesfulDirectLinkCounter = 0;
	neighbourCounter = 0;
	successfulDirectLink = false;
	successfulNeighbourLink = false;
	penultimateMoveWasLink = false;
	linksLearnt = 0;
	linksUpdated = 0;
	possibleLinkstoLearn = 0;
	//endstats
}

void eyeSaccading::getStats(int *sac, int *stc, bool *sdl, bool *snl, int *udlc, int *nc,
		bool *pmwl, int *ll, int *lu, int *pll)
{

	*sac =  saccadeCounter;
	*stc =  stepCounter;
	*sdl =  successfulDirectLink;
	*snl =  successfulNeighbourLink;
	*udlc =  unsuccesfulDirectLinkCounter;
	*nc =  neighbourCounter;
	*pmwl =  penultimateMoveWasLink;
	*ll = linksLearnt;
	*lu = linksUpdated;
	*pll = possibleLinkstoLearn;

}

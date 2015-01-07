/*
 * handEyeCoordination.cpp
 *
 *  Created on: 17 Aug 2011
 *      Author: icub
 */

#include "handEyeCoordination.h"

using namespace yarp::os;

handEyeCoordination::handEyeCoordination()
{
	gm = new GazeMap();
	initLogs();

	if(params.m_LOAD)
		loadGazeReachMap(params.m_FILENAME);

}

handEyeCoordination::~handEyeCoordination()
{
}

void handEyeCoordination::init(EyeHeadSaccading* pehCont, armReaching* par, Target* ptarget)
{
	ehCont = pehCont;
	ar = par;
	target = ptarget;

}


bool handEyeCoordination::stationary()
{
	return ar->getArmController()->armsStationary()  && ehCont->allStationary();
}

bool handEyeCoordination::handCentred()
{
	double targX, targY;
	if(target->targetCentred(&targX, &targY,YELLOW))
	{
		ehCont->centreEyesInHead();
		//hand is likely to be centred in view.
		return true;
	}
}
string handEyeCoordination::handVisible(double* targX, double* targY, bool* success)
{
	string colour="none";
//	if(target->getTarget(targX, targY, GREEN))
//	{
//		colour = GREEN;
//		*success = true;
//	}
//	else
	if(target->getTarget(targX, targY, YELLOW))
	{
		colour = YELLOW;
		*success = true;
	}
	else
	{
		cout << "Neither Yellow or Green Targets are visible from here" << endl;
		*success = false;
	}
	return colour;
}

bool handEyeCoordination::findArm(ReachField* reach, bool rightArm)
{
	double targX, targY;
//	pos->positionMove(0, -38);		//approximately gaze -20, -65?
//	pos->positionMove(2,  10);
//	ehCont->goToGazeField(-20,-65);

	//Look to see if we already know how to look at the arm
	bool knownPos = checkArmLinks(reach);
	if(knownPos)
	{

		logEntry("Recognised arm configuration and moved head to matching gaze");
		logField(reach);
		//Check hand centred
		if(handCentred())
			return true;
		else
			logEntry("Attempted to follow gaze-reach link but it didn't seem to work");

	}


	int attemptCounter = 0;
	while(attemptCounter < 15)
	{
		//if no target is visible, make a random eye+head movement.
		//	- prefer positions recently recorded in gaze map?
		while(!(ehCont->allStationary()))
		{
			Time::delay(0.5);	//ensure gaze is stationary
		}
		string colour;

		bool visible;
		colour = handVisible(&targX, &targY, &visible);
//		int x=-20;
//		int y=-65;
		if(!visible)
		{
			logEntry("Neither Yellow or Green Targets are visible from here");
//			ehCont->goToGazeField(x,y);
//			x-=5;
//			y-=5;
			//ehCont->babble();	//blocks till movement complete
			attemptCounter++;
			continue;
		}

		logEntry("Found a target which may be a hand, now attempting to fixate on it");



//		if(target->matchColour(YELLOW) || target->matchColour(GREEN))
//		{ // check to see if the colour of the target matches a hand
			//now need to fixate on the target

		int counter =0;
		bool fixated = false;
		while(counter<15)
		{
//			target->getTarget(&targX, &targY, colour);	//Fixation is being handled by tracker thread at the moment
			if(target->targetCentred())
			{
				Time::delay(0.5);
				break;
			}
			//fixated = ehCont->fixate(targX, targY,colour, true);	//try to use eye+head links first
			counter ++;
		}


		if(target->targetCentred())
		{
			Time::delay(0.5);
			return true;
		}
		else
		{
			logEntry("Failed to fixate on the hand");
			//ehCont->babble();	//blocks till movement complete
		}


//			//Try to fixate on a green target
//			e->saccade(-2, targX, targY, GREEN);
//			if(target->targetCentred(&targX, &targY, GREEN))
//				return true;

//		}
//		else
//		{
//			cout << "A target is visible, but it is not one of the hands" << endl;
//		}
//		e->makeRandomMove();
//		h->makeRandomMove();	//blocks till movement complete
		attemptCounter++;
	}

	logEntry("Unable to find the hands after 10 attempts");
	return false;
}

/**
 * visually Locates hand once then adds link between gaze and reach fields
 */
bool handEyeCoordination::handGaze(bool rightArm)
{
	double startTargX, startTargY;

//	bool rightArm = true;
	//make a random arm movement (record which arm moved)
	ar->learnReach(rightArm);
//	bool rightArm = a->babble(a->LEFT);	//blocks till complete

	string colour = YELLOW;
//	target->getTarget(&startTargX, &startTargY, colour);


	//look for reach field and see if its linked to a gaze field
	ReachField* reach = new ReachField();
	if(ar->isReachField(rightArm))
		reach = ar->getReachField(rightArm);
	else
	{
		double* armConf = new double[16];
		ar->getArmController()->getCurrentPosition(armConf, rightArm);
		gm->addReachField(armConf, rightArm);
		reach = gm->getReachField(armConf, rightArm);
//		logEntry("Failed to obtain reach field");
//		return false;
	}

	logEntry("Reach Field:");
	logField(reach);



	bool success = findArm(reach, rightArm);
	if(!(success || target->targetCentred()))
	{
		logEntry("Failed to visually locate arm");
		return false;
	}
	while(!stationary())
	{
		Time::delay(0.2);
	}

	Time::delay(1.2);


	//Should really confirm that we are fixated on the hand here before adding the link
	GazeField* gf;
	if(ehCont->isGazeField())
		gf = ehCont->getGazeField();
	else
	{
		if(ehCont->addGazeField())
			gf = ehCont->getGazeField();
		else
		{
			logEntry("Failed to obtain gaze field");
			return false;
		}
	}
	logEntry("Gaze Field:");
	logField(gf);

	//get Depth of hand in gaze space
	double depth;
	ehCont->verge(colour, &depth, true);	//depth returned is the vergence angle.
//	ehCont->getDepth(colour, &depth);
	printf("Estimating the depth (vergence angle) at: %.2f\n",depth);


	HeadConfig* hc = gf->getPreferredGaze();
	ReachConfig* rc;
	if(rightArm)
		rc = reach->getRightReach();
	else
		rc = reach->getLeftReach();

	double* hmc = new double[6];
	double* amc = new double[16];
	hc->toArray(hmc);
	rc->toArray(amc);

	cout<< "Adding coordination between " << *hc << " and " << *rc << endl;
	gm->addGazeReach(hmc, amc, rightArm, depth);

	//gm->addCoordination(gf, reach);	//bug on getting reach config, but works fine for addGazeReach


//		for(int i=1; i<10; i++)
//		{
////			a->smallBabble(rightArm);
//			a->babble(rightArm);
//			success = findArm(armConf, rightArm);
//			if(success)
//			{
//				gotEnc = a->getCurrentPosition(armConf, rightArm);
//				gotEnc &= enc->getEncoders(headMotorPos);
//				if(gotEnc)
//				{
//					gm->addGazeReachField(headMotorPos, armConf, rightArm);
//				}
//				else
//				{
//					cout << "failed to obtain one of the encoders" << endl;
//				}
//			}
//		}

//	}
//	else
//	{
//		//not fixated on the hand that just moved, need to fixate on the other hand.
//		//could the hand have moved in such a way that the target is still in the centre?
//		cout << "Made a small hand movement after fixating on a yellow/green target, " <<
//				"however the target still seems to be central.  Either we were looking " <<
//				"at the other hand, or the movement was insufficient to move it from the fovea." << endl;
//	}

	//find target that is yellow or green
	//record position for addition to gaze map
	//make a small movement of the same hand
	//if target moves then add first reach-gaze configuration for that hand
	//else move other hand, and if target moves add first reach-gaze configuration for second hand
	//saccade to new hand position
	//add new reach-gaze configuration
	//make SACCADE/10 small hand movements

	return success;
}



bool handEyeCoordination::checkArmLinks(ReachField* rf)
{
	cout << "Checking for arm links" << endl;
	GazeField* gf = new GazeField();

	bool success = gm->getEyeCoordination(rf, &gf);
	if(success)
	{
		cout << "Found matching arm link" << endl;
		success = ehCont->goToGazeField(gf);
	}
	else
		cout << "No matching arm link found" << endl;

	return success;
}


int handEyeCoordination::learnCoordination(bool rightArm)
{
	int counter = 0;
	int failed = 0;
	while (counter < NUM_POSES)
	{
		startCoordLog(counter+failed);
		cout << "attempt " << counter << endl;
		bool success = handGaze(rightArm);
		if(success)
			counter ++;
		else
		{
			cout << "trying again" << endl;
			failed++;
		}
	}
	return counter;
}

bool handEyeCoordination::addLink(GazeField* gf, ReachField* rf, double depth)
{
	gm->addCoordination(gf, rf, depth);
}

bool handEyeCoordination::updateLink(ReachField* rf, GazeField* newGF, double depth)
{
	gm->updateLink(rf, newGF, depth);
}



void handEyeCoordination::reachTo(ReachField* reach)
{
	ar->goToReachField(reach);

}



GazeMap* handEyeCoordination::getGazeMap()
{
	return gm;
}

bool handEyeCoordination::saveGazeReachMap()
{
	Gaze_IO gmIO;
	try{
		cout << "There are " << gm->getNumGazeFields() << " gaze fields to save" << endl;
		gmIO.saveMappingToXML(gm, params.m_PATH+"GM_" + params.m_FILENAME);
		cout << "Successfully saved: ";
		cout << gm->getNumGazeFields() << " Gaze fields, ";
		cout << gm->getNumReachFields() << " Reach fields and ";
		cout << gm->getNumLinks() << " links " << endl;
	}
	catch(IMapException ime)
	{
		cout << "An error occurred whilst attempting to save the gaze map"<<endl;
	}
//	if(gmIO.saveMappingToXML(gm, path+"GM_" + filename))	//gm_io adds type and .xml to filename
//		cout << gm->getNumGazeFields() << " Gaze map fields successfully saved" << endl;
//	else
//		cout << "An error occurred whilst attempting to save the gaze map"<<endl;
	return true;
}

bool handEyeCoordination::loadGazeReachMap(string filename)
{
	Gaze_IO gm_io;
	delete gm;
	gm = gm_io.loadMappingFromXML(params.m_PATH + "GM_" + filename);
//	cout << "There are " << gm->getNumGazeFields() << " fields in the gaze map" << endl;

	string str = "There are ";
	str = appendInt(str, gm->getNumGazeFields());
	str += " gaze fields in the gaze map";
	logEntry(str);

	return true;
}




bool handEyeCoordination::hasLinkedReach(GazeField* gaze, double depth)
{
	ReachField* rf = new ReachField();
	return gm->getHandCoordination(gaze, &rf, depth);
}
bool handEyeCoordination::hasLinkedGaze(ReachField* reach)
{
	GazeField* gf = new GazeField();
	return gm->getEyeCoordination(reach, &gf);
}
ReachField* handEyeCoordination::getLinkedReach(GazeField* gaze, double depth)
{
	ReachField* rf = new ReachField();
	gm->getHandCoordination(gaze, &rf, depth);
	return rf;
}
ReachField* handEyeCoordination::getLinkedReach(GazeField* gaze)
{
	ReachField* rf = new ReachField();
	gm->getHandCoordination(gaze, &rf);
	return rf;
}

GazeField* handEyeCoordination::getLinkedGaze(ReachField* reach)
{
	GazeField* gf = new GazeField();
	gm->getEyeCoordination(reach, &gf);
	return gf;
}



bool handEyeCoordination::getNearestReachForGaze(GazeField* gf, ReachField** rf, double* dist)
{
	//First look for reach fields in same gaze direction, regardless of depth
	//Then consider depth.  Note: for safety, it may be worth looking for a depth that is less than the target depth...


	printf("Gaze(%.2f, %.2f)\n", gf->getXcoord(), gf->getYcoord());

	double *headMotorConfig = new double[6];
	HeadConfig *hc = gf->getPreferredGaze();
	hc->toArray(headMotorConfig);


	vector<GazeReachLink*> fields = gm->getReachSetForGaze(headMotorConfig, 5);
	*dist = 2000;
	ReachField* closest;
	printf("There are %i links in that gaze direction\n", fields.size());
	if(fields.size()==0)
		gm->getNearestReachFieldForGaze(headMotorConfig, &closest, dist);
	else{
//	for(size_t i=0; i<fields.size(); i++)
//	{
//		GazeReachLink* l = fields.at(i);
//		if(abs(l->gazeDepth - depth)< *dist)
//		{
//			*dist = abs(l->gazeDepth - depth);
//			printf("Link at depth %.2f\n", l->gazeDepth);
//			closest = l->reach;
//		}
//	}

		sort(fields.begin(), fields.end(), compareGazeLinks);
		closest = fields.at(0)->reach;
	}
	*rf = closest;
	return true;

//	if(*dist != 2000)
//	{
//		*rf = closest;
//		return true;
//	}
//	else
//	{
//		*dist = 0;
//		return false;
//	}

}


/**
 * Dist is the gaze distance
 */
bool handEyeCoordination::getNearestReachForGaze(GazeField* gf, ReachField** rf, double depth, double* dist)
{
	//First look for reach fields in same gaze direction, regardless of depth
	//Then consider depth.  Note: for safety, it may be worth looking for a depth that is less than the target depth...


	printf("Gaze(%.2f, %.2f)\n", gf->getXcoord(), gf->getYcoord());

	double *headMotorConfig = new double[6];
	HeadConfig *hc = gf->getPreferredGaze();
	hc->toArray(headMotorConfig);


	vector<GazeReachLink*> fields = gm->getReachSetForGaze(headMotorConfig, 10);
	*dist = 2000;
	ReachField* closest;
	printf("There are %i links in that gaze direction\n", fields.size());
	if(fields.size()==0)
		gm->getNearestReachFieldForGaze(headMotorConfig, &closest, dist);

	sort(fields.begin(), fields.end(), compareGazeLinks);
	for(size_t i=0; i<fields.size(); i++)
	{
		*dist = 2000;
		GazeReachLink* l = fields.at(i);
		if((l->gazeDepth-depth) > -3)
		{
			*rf = l->reach;
			*dist = sqrt(pow(l->gaze->getXcoord()-gf->getXcoord(),2) + pow(l->gaze->getYcoord()-gf->getYcoord(),2));
			return true;
		}

		if(abs(l->gazeDepth - depth)< *dist)
		{
			*dist = abs(l->gazeDepth - depth);
			printf("Link at depth %.2f\n", l->gazeDepth);
			closest = l->reach;
		}
	}
	if(*dist != 2000)
	{
		*rf = closest;
		return true;
	}
	else
	{
		*dist = 0;
		return false;
	}

}


bool handEyeCoordination::getNearestLinkForGaze(GazeField* gf, GazeReachLink** link, double depth, double* dist)
{
	//First look for reach fields in same gaze direction, regardless of depth
	//Then consider depth.  Note: for safety, it may be worth looking for a depth that is less than the target depth...


	printf("Gaze(%.2f, %.2f)\n", gf->getXcoord(), gf->getYcoord());

	double *headMotorConfig = new double[6];
	HeadConfig *hc = gf->getPreferredGaze();
	hc->toArray(headMotorConfig);


	vector<GazeReachLink*> fields = gm->getReachSetForGaze(headMotorConfig, 10);
	*dist = 2000;
	GazeReachLink* closest;
	printf("There are %i links in that gaze direction\n", fields.size());
	if(fields.size()==0)
	{
		ReachField* rf;
		gm->getNearestReachFieldForGaze(headMotorConfig, &rf, dist);
		closest = gm->getLinkFromReach(rf);
	}
	for(size_t i=0; i<fields.size(); i++)
	{
		GazeReachLink* l = fields.at(i);
		if(abs(l->gazeDepth - depth)< *dist)
		{
			*dist = abs(l->gazeDepth - depth);
			printf("Link at depth %.2f\n", l->gazeDepth);
			closest = l;
		}
	}
	if(*dist != 2000)
	{
		*link = closest;
		return true;
	}
	else
	{
		*dist = 0;
		return false;
	}

}




vector<GazeReachLink*> handEyeCoordination::getTableReaches()
{
	vector<GazeReachLink*> allLinks = gm->getLinks();
	vector<GazeReachLink*> tableLinks;

	for(size_t i=0; i<allLinks.size(); i++)
	{
		GazeReachLink* l = allLinks.at(i);
		if(l->getArmAngle() < 100 && l->gazeDepth > 33)
		{
			tableLinks.push_back(l);
		}
	}
	return tableLinks;
}




//**************************************
// LOGGING

void handEyeCoordination::initLogs()
{
	string fullpath = params.m_PATH + "handEyelog.txt";
	handEyelog.open(fullpath.c_str());
}
void handEyeCoordination::logEntry(string message)
{
	cout << message << endl;
	handEyelog << message << endl;
}

void handEyeCoordination::logField(const Field* field)
{
	cout << *field << endl;
	handEyelog << *field << endl;
}


void handEyeCoordination::startCoordLog(int counter)
{
	handEyelog << "*******************************" << endl;
	handEyelog << "Starting Coordination attempt: "<< counter << endl;
}

void handEyeCoordination::closeLogs()
{
	handEyelog << "**************END**************" << endl;
	handEyelog.close();
}






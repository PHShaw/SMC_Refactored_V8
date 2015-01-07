/*
 * torsoSaccading.cpp
 *
 * NOTE: eye, head and torso saccading share a lot of common methods... these could be taken out
 *  and then extended on with specifics where there is variation
 *
 *  //TODO: Figure out how to reuse torso movements
 *
 *  Created on: 8 Jun 2011
 *      Author: icub
 */

#include "torsoSaccading.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

torsoSaccading::torsoSaccading(EyeHeadSaccading* pEhcont, Target* pTarget)
{
	torso = new torsoController(params.robot);
	cout << "Torso controller initialised" << endl;
	ehCont = pEhcont;
	target = pTarget;

	tc = new torsoCompensator();
	initCompensator();
	cout << "Torso compensator initialised" << endl;

	headSac = ehCont->headSacker();


	if(!params.load)
		torso_ppm = new ffm(POLAR_MAP, -310.f, 630.f, -270.f, 510.f, POLAR_MAP,
										-101.f, 101.f, -61.f, 61.f);
	else
		loadMapping(params.filename);


	openLogs();
	srand(NULL);

}

torsoSaccading::~torsoSaccading()
{
	delete torso;
	delete torso_ppm;
}

bool torsoSaccading::loadMapping(string filename)
{
	bool success;
	FFM_IO io;
	try{
		torso_ppm = io.loadMappingFromXML(params.path + "torso_" + filename +".xml");
		cout << "There are " << torso_ppm->getNumLinks() << " links in the torso map"<< endl;
		success = true;
	}
	catch(IMapException ime)
	{
		cout << "Error trying to load torso mappings from: " << endl;
		cout << params.path << "torso_" << filename << ".xml" << endl;
		cout << "Generating blank mapping" << endl;
		torso_ppm = new ffm(POLAR_MAP, -310.f, 630.f, -270.f, 510.f, POLAR_MAP,
										-101.f, 101.f, -61.f, 61.f);
		success=false;
	}
	return success;
}

bool torsoSaccading::saveMapping()
{
	bool success;
	FFM_IO io;
	try{
		io.saveMappingToXML(torso_ppm,params.path + "torso_" + filename +".xml");
		cout << torso_ppm->getNumLinks() << " Torso links successfully saved"<<endl;
		success=true;
	}
	catch(IMapException ime)
	{
		cout << "An error occurred whilst attempting to save the torso links"<<endl;
		success = false;
	}
	return success;
}


bool torsoSaccading::allStationary()
{
	return torso->stationary() && ehCont->allStationary();
}


//Used to record the gaze direction prior to making the torso movement
void torsoSaccading::recordPrePositions()
{
	while(!allStationary())
		Time::delay(0.5);

	ehCont->getGazeDirection(&gazeX1, &gazeY1);


	torso->getCurrentPosition(&torsoX1, &torsoY1);

	string entry = "Pre-direction of gaze is (";
	entry = appendInt(entry, (int)gazeX1);
	entry += ",";
	entry = appendInt(entry, (int)gazeY1);
	entry += ") and torso is (";
	entry = appendInt(entry, (int)torsoX1);
	entry += ",";
	entry = appendInt(entry, (int)torsoY1);
	entry += ")";
	logEntry(entry);
}


/**
 * Blocks until the move is completed.
 */
bool torsoSaccading::gotoField(Field* motorfield, bool compensate)
{

	logEntry("Moving to an existing linked field:");
	logField(motorfield);
	double relX = motorfield->getXcoord();		//relative motor values
	double relY = motorfield->getYcoord();

	//convert to actual motor values (or figure out why relativeMove doesn't work)

	double curX, curY;
	torso->getCurrentPosition(&curX, &curY);

	double x, y;
	y = relY+curY;
	x = relX+curX;

	logEntry("Actual movement is ", x ,y);
	if(compensate)
	{
		tc->resetOverflow();
		boost::thread tcThrd(boost::bind(&torsoCompensator::track,tc));
		torso->move(x, y, true);
		Time::delay(1);
		tcThrd.interrupt();
	}
	else
		torso->move(x, y, true);


	return true;
}


void torsoSaccading::compensateToRest(){
	tc->resetOverflow();
	boost::thread tcThrd(boost::bind(&torsoCompensator::track,tc));
	torso->move(0, 0, true);
	Time::delay(2);
	tcThrd.interrupt();
}

bool torsoSaccading::makeLink(double startX, double startY, double endX, double endY,
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


bool torsoSaccading::makeLink(double startX, double startY, double endX, double endY, PolarField* motor)
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
bool torsoSaccading::makeLink(double inputX, double inputY, double motorX, double motorY)
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

bool torsoSaccading::makeLink(PolarField* source, PolarField* motor)
{
	logEntry("Making a torsolink between fields: ");
	logField(source);
	logField(motor);


	if(torso_ppm->containsLink(source, motor))
	{
//		source->useField();
//		motor->useField();
		logEntry("strengthening existing link, rather than adding new link");
		return true;
	}

	bool ok = torso_ppm->addLink(source, motor);
	source->setCalcdLinks(100);
	motor->setCalcdLinks(100);
	return ok;
}




bool torsoSaccading:: calcLink(string colour)
{
	while(!allStationary())
		Time::delay(0.1);


	double gazeX2, gazeY2;	//Eye position after torso compensation
	ehCont->getGazeDirection(&gazeX2, &gazeY2);

	double TorsoX2, TorsoY2;
	torso->getCurrentPosition(&TorsoX2, &TorsoY2);

	string entry = "Final direction of gaze is (";
	entry = appendInt(entry, (int)gazeX2);
	entry += ",";
	entry = appendInt(entry, (int)gazeY2);
	entry += ") and torso is (";
	entry = appendInt(entry, (int)TorsoX2);
	entry += ",";
	entry = appendInt(entry, (int)TorsoY2);
	entry += ")";
	logEntry(entry);

	double gazeShiftX, gazeShiftY;		//relative gaze shift (in eye motor coordinates)
	gazeShiftX = gazeX2-gazeX1;
	gazeShiftY = gazeY2-gazeY1;

	double torsoSaccadeX, torsoSaccadeY;	//magnitude and direction of torso movement
	torsoSaccadeX = TorsoX2 - torsoX1;		//relative motor values for torso movement
	torsoSaccadeY = TorsoY2 - torsoY1;

	double headX, headY;				//gaze shift converted to head motor coordinates
	headX = eyeXtoHeadX(gazeShiftX);
	headY = eyeYtoHeadY(gazeShiftY);

	double targetX, targetY;			//Look up motor movement in head map to look for linked target position


	bool success;

	if(headSac->isMotorField(headX, headY))
	{
		PolarField* motor;
		float dist=0;
		if(nearestNeighbour)
		{
			motor = headSac->getNearestLearntOutput(headX, headY, &dist);
			logEntry("nearest learnt output at distance ", dist, motor->getUsage());
		}
		else
		{
			motor = headSac->getMotorField(headX, headY);
			dist = motor->getRadius();
		}
		float radius = motor->getRadius();
		if(dist<(radius*3) && dist>0 && motor->getUsage()>=0)
		{
			logEntry("Distance and usage: ", dist, motor->getUsage());


			if(headSac->isLinkedInput(motor))
			{
				PolarField* sensor = headSac->getLinkedInput(motor);
				targetX = sensor->getXcoord();
				targetY = sensor->getYcoord();
				logEntry ("Target is at ", targetX, targetY);

//				torsoChain.push_front(position(-torsoSaccadeX, -torsoSaccadeY, targetX, targetY));

				cout << "Attempting to learn link based on torso only movement" << endl;
				double targX, targY, dist;
				bool centred = target->targetCentred(&targX, &targY,colour);
				target->fovea(targX, targY, &dist);
				if(centred)
				{
					success = makeLink(targetX, targetY, targX, targY, -torsoSaccadeX, -torsoSaccadeY);


					if(success)
					{
						logEntry("link added based on torso only movement");
					}
					else
					{
						logEntry("Failed to add link based on torso only movement");
					}
				}
				else if(dist<32)
				{
					logEntry("nearly centred target, so attempting to add link anyway");
					success = makeLink(targetX, targetY, targX, targY, -torsoSaccadeX, -torsoSaccadeY);
					if(success)
					{
						logEntry("link added based on torso only movement");
					}
					else
					{
						logEntry("Failed to add link based on torso only movement");
					}
				}
				else
				{
					logEntry("Target not centred so not adding a link");
					success=false;
				}

			}
			else
			{
				logEntry("No linked head input for relative motor change for torso link calculation, ",
									motor->getXcoord(), motor->getYcoord());

//				torsoChain.push_front(position(-torsoSaccadeX, -torsoSaccadeY, 0, 0));

				success = false;
			}
		}
		else
		{
			logEntry("Motor field is too far away or unreliable");
			logField(motor);
//			torsoChain.push_front(position(-torsoSaccadeX, -torsoSaccadeY, 0, 0));
			success = false;
		}
	}
	else
	{
		logEntry("No head motor field for ", headX, headY);
//		torsoChain.push_front(position(-torsoSaccadeX, -torsoSaccadeY, 0, 0));
		success = false;
	}


	return success;

}



/*
 * Whilst restricting the torso to small babble movements, only the central fields can be learnt.
 * In order to give a better over all mapping, it might be possible to chain torso movements together
 * as is done in the eye saccading to learn more links.
 * Therefore, each time a movement is made, it is linked back to previous movements.  A limit
 * can be applied to the 'memory' of previous positions
 *
 * Uses structs position and relativeMovement from eyeSaccading.h
 */
int torsoSaccading::learnMapping(string colour)
{
	double startYaw, startPitch;
	torso->getCurrentPosition(&startYaw, &startPitch);

	double targX, targY;
	bool centred = target->targetCentred(&targX, &targY, colour);

	if(centred)
		torsoChain.push_front(position(startYaw, startPitch, targX, targY));
	else
		return 0;


	double dist;
	int linksTotal;
	for(int i=0; i<20; i++)
	{
		centred = target->targetCentred(&targX, &targY, colour);
		target->fovea(targX,targY,&dist);
		bool reset=false;
		while(dist>32)
		{
			ehCont->eyeHeadSaccade();
			colour = target->getColour();
			centred = target->targetCentred(&targX, &targY, colour);
			target->fovea(targX,targY,&dist);
			reset=true;
		}
		if(reset)
		{
			torsoChain.clear();
//			torso->toRest();
			reset = false;
		}


		cout << "************Starting new torso movement " << i << "*************" << endl;
		saccade(i,colour);


		int links = 0;
		if(torsoChain.size()> 10)
		{
			torsoChain.pop_back();
		}

//		links = learnChain();
		linksTotal += links;
		cout << "Learnt " << links << " links from this chaining" << endl;

		Time::delay(3);
	}

	return linksTotal;
}


int torsoSaccading::learnChain()
{
	int linkCounter = 0;
	list<position>::iterator it;
	cout << "** Attempting to learn torso chain of size "<< torsoChain.size() <<" **" << endl;

	relativeMovement rel(torsoChain.front().eyeMotorX, torsoChain.front().eyeMotorY);

	it=torsoChain.begin();
	it++;
	for(; it!=torsoChain.end(); it++)
	{
		cout <<"***"<< endl;
		position p = *it;
		cout << "Position: " << p.eyeMotorX << ", " << p.eyeMotorY << ", " << p.targetX << ", " << p.targetY << endl;
		rel -= p;
		if(p.targetX!=0 && p.targetY!=0)
		{
			cout << "Relative movement: " << rel.relativeX << ", " << rel.relativeY << endl;

			bool success = makeLink(p.targetX, p.targetY, rel.relativeX, rel.relativeY);
			if(success)
				linkCounter++;
		}
	}
	return linkCounter;
}

//void torsoSaccading::startCompensator()
//{
//	tcThrd.thread(boost::bind(&torsoCompensator::track,tc));
//}
//
//void torsoSaccading::stopCompensator()
//{
//	tcThrd.interrupt();
//}

/**
 * The makes a random torso movement and learns a link.  It does not look for any
 * pre-existing links.
 */
bool torsoSaccading::saccade(int saccadecounter, string colour)
{
	while(!allStationary())
		Time::delay(0.5);

	recordPrePositions();			//record the gaze direction before eye and head
									// starts compensating for torso movement


	tc->resetOverflow();
	boost::thread torThrd(boost::bind(&torsoCompensator::track,tc));

	cout << "making random torso movement" << endl;
	double startX, startY;
	torso->getCurrentPosition(&startX, &startY);
	torso->babble();	//blocks till movement complete
	double x,y;
	torso->getCurrentPosition(&x, &y);

	double relX = x - startX;
	double relY = y - startY;
	motorRecord(relX, relY, saccadecounter);

	logEntry("Made a random movement to actual ", x,y);

	Time::delay(3);	// Allow time for VOR to adjust
	while(!allStationary())
		Time::delay(1);

	torThrd.interrupt();


	double targX, targY;
	target->getTarget(&targX, &targY, colour);
	ehCont->fixate(targX, targY, colour, false);

//	target->getTarget(&targX,&targY);	//Target should be central

//	bool success = true; //calcLink(v);	//just testing basic links at the moment, to try and get that right
	bool success = false;
	if(learn)
		success = calcLink(colour);
	return success;
}

//No learning, single movement saccade - no guarantee of fixation
//Aim is that this can be used to centre the eyes in torso on target
//So should be dealing with eye position, rather than target location
//TODO
//bool torsoSaccading::simpleSaccade()
//{
//
//	double eyeX, eyeY;
//	eye->getCurrentPosition(&eyeX, &eyeY);
//	PolarField* eyeMfield = new PolarField();
//	bool gotField = eyeSac->isMotorField(eyeX, eyeY);
//	if(gotField)
//		eyeMfield = eyeSac->getMotorField(eyeX, eyeY);
//
//	PolarField* eyeRfield = new PolarField();
//	gotField &= eyeSac->isLinkedInput(eyeMfield);
//	if(gotField)
//		eyeRfield = eyeSac->getLinkedInput(eyeMfield);
//	else
//	{
//		logEntry("Failed to obtain eye input field for simple torso saccade");
//		torso->babble();
//	}
//
//	double targX, targY;
//	targX = eyeRfield->getXcoord();
//	targY = eyeRfield->getYcoord();
//
//	PolarField* torsoRfield = new PolarField();
//	gotField = isRetinaField(targX, targY);
//	if(gotField)
//		torsoRfield = getRetinaField(targX, targY);
//	else
//	{
//		logEntry("Failed to obtain torso input field for simple saccade");
//		torso->babble();
//	}
//
//	PolarField* torsoMfield = new PolarField();
//	gotField = isLinkedOutput(torsoRfield);
//	bool success = false;
//	if(gotField)
//	{
//		torsoMfield = getLinkedOutput(torsoRfield);
//		success = gotoField(torsoMfield);
//	}
//
//	else
//	{
//		logEntry("Input field  not linked for simple torso saccade");
//		torso->babble();
//	}
//
//	if(!success)
//	{
//		logEntry("Target is not reachable for simple torso saccade");
//	}
//
//	//colour can be centred without eyes being centred
//	//this success measure should be related to eye position
//	eye->getCurrentPosition(&eyeX, &eyeY);
//	eyeX = abs(eyeX);
//	eyeY = abs(eyeY);
//
//	if(eyeX <= 5 && eyeY <= 5)
//	{
//		return true;
//	}
//	else
//		return false;
//
//}


/**
 * This function calculates a torso movement that should aim to straighten up the eye and head
 */
bool torsoSaccading::easeHead()
{
	double gazeX, gazeY;
	ehCont->getGazeDirection(&gazeX, &gazeY);

//	cout << "Starting from gaze direction: " << gazeX << ", " << gazeY << endl;

	double headX, headY;				//gaze converted to head motor coordinates
	headX = eyeXtoHeadX(gazeX);
	headY = eyeYtoHeadY(gazeY);

	double currentTorsoX, currentTorsoY;	//Yaw and pitch respectively
	torso->getCurrentPosition(&currentTorsoX, &currentTorsoY);


	//Options for proportional formulae:
	// f(x)=-0.015x^2 +100			//gradual fade away from 100
	// f(x)=-0.00001x^4 +100		//level around 100 with sharper fall off from about 20 onwards
	// f(x)=-0.000025x^4 + 0.04x^2 + 84 //dip at 0, going to 100% around 30 then falling off sharply
	//−(0.09x−0.9)^4+100	//shifted along the x axis for the headY
	double proportionX, proportionY;

	//x: −cos(0.06x)∙50+50
	//y: −cos(0.06x−1)∙50+50

	proportionX = -50*cos(0.06*headX)+50;	//gives % of gaze that should be compensated for using the torso, if possible
	proportionY = -50*cos(0.06*headY-1)+50;

//	cout << "Adjusting by proportion: " << proportionX << ", " << proportionY << endl;




	double partX, partY;	//assuming torso at 0, this would be the desired movement
//	partX = headX*proportionX/100;
//	partY = headY*proportionY/100;

	partX = headX;
	partY = headY;


	//head to torso is approximately 1:1
	double newTorsoX, newTorsoY;
	newTorsoX=currentTorsoX+partX;
	newTorsoY=currentTorsoY+partY;

	tc->resetOverflow();
	boost::thread torThrd(boost::bind(&torsoCompensator::track,tc));
	torso->move(newTorsoX,newTorsoY,true);

	while(!allStationary())
		Time::delay(1);

	torThrd.interrupt();

	ehCont->getGazeDirection(&gazeX, &gazeY);

//	cout << "Finishing from gaze direction: " << gazeX << ", " << gazeY << endl;

	return true;

}


//Doesn't block
bool torsoSaccading::followLink(double relX, double relY)
{
	double x, y;
	torso->getCurrentPosition(&x, &y);


	//Convert to actual motor values
	x += relX;
	y += relY;

	torso->move(x,y, false);

	logEntry("Following a link with motor values provided by gazeController");
}






/* *****************************************************************
 *
 * 		FIELD ACCESS METHODS
 *
 * *****************************************************************
 */

bool torsoSaccading::isRetinaField(double x, double y)
{
	logEntry("Requesting retina field at: ",x,y);
	PolarField* field = (PolarField*)torso_ppm->getInputField((float)x, (float)y);
	if(torso_ppm->isNull(field))
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
PolarField* torsoSaccading::getRetinaField(double x, double y)
{
	return (PolarField*)torso_ppm->getInputField((float)x, (float)y);
}

bool torsoSaccading::isMotorField(double x, double y)
{
	logEntry("Requesting motor field at: ",x,y);
	PolarField* field = (PolarField*)torso_ppm->getOutputField((float)x, (float)y);
	if(torso_ppm->isNull(field))
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
PolarField* torsoSaccading::getMotorField(double x, double y)
{
	return (PolarField*)torso_ppm->getOutputField((float)x, (float)y);
}


bool torsoSaccading::isLinkedInput(PolarField* motorField)
{
	logEntry("Requesting linked retina field for field:");
	logField(motorField);
	PolarField* retinaField = (PolarField*)torso_ppm->getLinkedInput(motorField);
	if(torso_ppm->isNull(retinaField))
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
PolarField* torsoSaccading::getLinkedInput(PolarField* motorField)
{
	return (PolarField*)torso_ppm->getLinkedInput(motorField);
}

bool torsoSaccading::isLinkedOutput(PolarField* retinaField)
{
	logEntry("Requesting linked motor field for field:");
	logField(retinaField);
	PolarField* motorField = (PolarField*)torso_ppm->getLinkedOutput(retinaField);
	if(torso_ppm->isNull(motorField))
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
PolarField* torsoSaccading::getLinkedOutput(PolarField* retinaField)
{
	return (PolarField*)torso_ppm->getLinkedOutput(retinaField);
}



PolarField* torsoSaccading::getNearestLearntInput(double targX, double targY, float* dist)
{
	return torso_ppm->getNearestLearntInputField((float)targX, (float)targY, dist);
}

PolarField* torsoSaccading::getNearestLearntOutput(double x, double y, float* dist)
{
	return torso_ppm->getNearestLearntOutputField((float)x, (float)y, dist);
}




//***************************************************************************
// LWPR torso model functions


void torsoSaccading::initMatlabPorts()
{
	//connect to the matlab ports
	cout << "Press any key when the matlab script is ready for connection." << endl;
	char c;
	cin >> c;

	Network yarp;
	portOut.open("/torso/write");
	portIn.open("/torso/read");
	bool ok = false;
	yarp.connect("/torso/write", "/matlab/read");
	yarp.connect("/matlab/write", "/torso/read");


	//send confirmation of connections to matlab, with filename for model
	Time::delay(0.5);
	Bottle& b1 = portOut.prepare();
	b1.clear();
	b1.addString(params.filename.c_str());
	portOut.write();

}

/**
 * It is possible that using the head in the gaze direction calculation is causing issues.  As a result,
 * the head should be kept centred, with just the eyes being used to fixate on targets.
 */
void torsoSaccading::LWPR_TorsoLearner(bool eyeAndHead)
{
	bool existing = false;
	char c;
	cout << "Does a model already exist [y/n]" << endl;
	cin >> c;
	if(c=='y')
		existing = true;



	int targCount=0, numTargets=50, numSamples=14;

	string modelName = "torsoModel";


	string fullpath = params.path + "torsoLWPRlog.txt";
	if(existing)
		LWPR_Logfile.open(fullpath.c_str(), ios::out | ios::app);	//append the contents onto the end of the file
	else
	{
		LWPR_Logfile.open(fullpath.c_str());
		LWPR_Logfile << "gazex gazey vergence torsoRot torsoTilt newVerg newTorRot newTorTilt"<< endl;
	}

	//if not got an existing model then need to initialise the matlab model.
	if(!existing)
	{
		cout << "Requesting LWPR initialisation" << endl;
		Time::delay(0.5);
		Bottle& b2 = portOut.prepare();
		b2.clear();
		b2.addInt(0);
		portOut.write();

		cout << "Waiting for confirmation of initialisation" << endl;
		input = portIn.read(true);
		existing = true;
		cout << "Confirmation received, proceeding with learning" << endl;
	}

	double targX, targY;
	string colour = "red";  //TODO make parameter


	while(targCount < numTargets)
	{

//		Request new starting position
//		send 1, followed by a desired number of training samples
		cout << "******************************************************" << endl;
		cout << "Requesting new starting point" << endl;
		Time::delay(0.5);
		Bottle& b2 = portOut.prepare();
		b2.clear();
		b2.addInt(1);
		b2.addInt(numSamples +1);
		portOut.write();

		cout << "Waiting for new starting point" << endl;
//		receive x, y for torso
		input = portIn.read(true);
		double rot, pitch;
		rot = input->get(0).asDouble();
		pitch = input->get(1).asDouble();

		printf("Starting point received at (%.2f, %.2f)\n", rot, pitch);

		cout << "Moving to new starting point" << endl;
//		move torso to x, y
		ehCont->toRest();
		torso->move(rot, pitch, true);


		cout << "Target " << targCount << ": Please move the target to a new location (that I can see) " <<
				"and type 'y' when ready (x to quit, or t to test)" << endl;
		cin >> c;
		if(c == 't')
		{
			LWPR_Tester(colour, eyeAndHead);

			cout << "Moving back to starting point" << endl;
//			move torso to x, y
			ehCont->toRest();
			torso->move(rot, pitch, true);

			cout << "Target " << targCount << ": Please move the target to a new location (that I can see) " <<
					"and type 'y' when ready (x to quit)" << endl;
			cin >> c;
		}


		if(c != 'y')
		{
//			cout << "Quitting" << endl;
//			Time::delay(0.5);
//			Bottle& b1 = portOut.prepare();
//			b1.clear();
//			b1.addInt(5);		//just quit
//			portOut.write();
//			portOut.close();
//			portIn.close();
			return;
		}

		cout << "Attempting to fixate on the target" << endl;
		bool visible = target->getTarget(&targX, &targY, colour);
		if(visible)
			ehCont->fixate(targX, targY, colour, false);

//		if(!fixated)
//			continue;

//		option 1:
//			fixate eyes on target
//			centre eyes in head
//			adjust torso to centre head on torso
//			check if target still fixated
//			fine tune target position to get it fixated
//		option 2:
//			centre eyes and head on torso
//			move target into a position fixated from starting torso position


		cout << "Attempting to centre the eyes and head on the torso" << endl;
		double depth, hx,hy, tx,ty;
		ehCont->centreEyesInHead();
//		string colour = target->getColour();
//		ehCont->verge(colour, &depth);
		ehCont->getEyeController()->move(0,0,true);
		ehCont->getEyeController()->verg(0, true);

		//TODO: Add function to centre head on torso

		ehCont->getHeadController()->getCurrentPosition(&hx, &hy);
		torso->getCurrentPosition(&tx,&ty);

		printf("Head starting from (%.2f, %.2f)\n", hx, hy);
		printf("Torso starting from (%.2f, %.2f)\n", tx, ty);
		//Torso Yaw to Head Yaw ~ 1:1
		//Torso Pitch to Head Pitch ~ 1:1 until head = +18, after which, eye takes over at f(x)=1.14x-17.4
		tx -= hx;
		ty -= hy;
		printf("Torso adjusting to (%.2f, %.2f)\n", tx, ty);
		torso->move(tx,ty, false);
		ehCont->getHeadController()->move(0,0,true);

//		verge on target
		cout << "Attempting to verge on the target" << endl;
		if(eyeAndHead)
			ehCont->verge(colour, &depth, true);
		else
			ehCont->verge(colour, &depth, false);

		cout << "Target " << targCount << ": Please move target so I'm focused on the target using only the torso" <<
				" and type 'y' when ready (or x to quit)" << endl;
		cin >> c;
		if(c != 'y')
		{
//			cout << "Quitting" << endl;
//			Time::delay(0.5);
//			Bottle& b1 = portOut.prepare();
//			b1.clear();
//			b1.addInt(5);		//just quit
//			portOut.write();
//			portOut.close();
//			portIn.close();
			return;
		}


//		record starting torso position and eye vergence
		cout << "Recording starting positions" << endl;
		double* headStartingPose = new double[6];
		double* torsoStartingPose = new double[3];

		ehCont->getHeadController()->getCurrentPosition(headStartingPose);
		torso->getCurrentPosition(torsoStartingPose);

		tracker* t = new tracker(colour);
		t->initTrack(ehCont->getMotorDriver(), target, ehCont);

		for(int i=0; i<numSamples; i++)
		{
			cout << "Sample " << i << " of target " << targCount << " requested" << endl;
			Time::delay(0.5);
//			3.	request new babble position
//				send 2
//				refixate eye (and head) on target
//				verge on target
			b2 = portOut.prepare();
			b2.clear();
			b2.addInt(2);
			portOut.write();

//			receive x, y for torso
			input = portIn.read(true);
			rot = input->get(0).asDouble();
			pitch = input->get(1).asDouble();
			printf("Babble position (%.2f,%.2f) received\n", rot, pitch);



			boost::thread trackThrd(boost::bind(&tracker::track,t)); 	//Attempt to track the target as the torso moves


//			move torso to new position
			cout << "Moving torso to new position" << endl;
			torso->move(rot, pitch, true);

			Time::delay(1.5);
			trackThrd.interrupt();		//stop tracking when torso has finished moving

//			cout << "Attempting to re-fixate gaze on target" << endl;
//			ehCont->getEyeController()->verg(0, true);
//			double targX, targY;
//			Time::delay(0.5);
//
			if(!eyeAndHead)
			{
				//Try to ensure the neck is zeroed, so only the eyes are in use.
				vor* v = ehCont->getVor();
				v->resetOverflow();
				boost::thread vorThrd(boost::bind(&vor::track,v));
				ehCont->getHeadController()->toRest();		//blocks, likely to overflow on eye range
				Time::delay(1.0);
				vorThrd.interrupt();
			}

			while(!ehCont->allStationary())
				Time::delay(0.5);


			Time::delay(0.5);
			target->getTarget(&targX, &targY, colour);
//			fixated = ehCont->fixate(targX, targY, colour, false);



			if(!target->targetCentred())
			{
				cout << "Sample " << i << " of target " << targCount <<
						". Failed to re-fixate gaze, please could you help me [y/n]" << endl;

				c='n';
			}
			else{
				cout << "Sample " << i << " of target " << targCount << endl; //". Please confirm that I am fixated [y/n]." << endl;
				c = 'y';
			}
//			cin >> c;

			if (c != 'y')
			{
				cout << "Oh well, will try again at the next position." << endl;
				continue;
			}





			cout << "Attempting to re-verge on target" << endl;
			if(eyeAndHead)
				ehCont->verge(colour, &depth, true);
			else
				ehCont->verge(colour, &depth, false);
			cout << "Please make sure I am verged on the target" << endl;
//			cin >> c;

			cout << "Calculating parameters for learning" << endl;
			double* headNewPose = new double[6];
			double* torsoNewPose = new double[3];
			ehCont->getHeadController()->getCurrentPosition(headNewPose);
			torso->getCurrentPosition(torsoNewPose);

			double gazeX, gazeY;
			if(eyeAndHead)
				ehCont->getGazeDirection(&gazeX, &gazeY);
			else
			{
				gazeX = headNewPose[4];
				gazeY = headNewPose[3];
			}


			double torsoRotChange, torsoPitchChange;
			torsoRotChange = torsoStartingPose[0] - torsoNewPose[0];
			torsoPitchChange = torsoStartingPose[2] - torsoNewPose[2];

//			4.	send training data
//				send 3, followed by current gaze direction, current vergence, current torso position, recorded vergence from starting position,
//					and difference between starting torso position and current torso position (starting - current)

			cout << "Sending training data" << endl;
			Time::delay(0.5);
			b2 = portOut.prepare();
			b2.clear();
			b2.addInt(3);
			b2.addDouble(gazeX);
			b2.addDouble(gazeY);
			b2.addDouble(headNewPose[5]);	//vergence
			b2.addDouble(torsoNewPose[0]);	//torso rotation
			b2.addDouble(torsoNewPose[2]);	//torso pitch
			b2.addDouble(headStartingPose[5]);	//starting vergence
			b2.addDouble(torsoRotChange);
			b2.addDouble(torsoPitchChange);
			portOut.write();

			LWPR_Logfile << gazeX << " " << gazeY << " " << headNewPose[5] << " " << torsoNewPose[0] << " " << torsoNewPose[2] << " ";
			LWPR_Logfile << headStartingPose[5] << " " << torsoRotChange << " " << torsoPitchChange << endl;

//			receive a default message when training data has been processed
			cout << "Waiting for confirmation from LWPR training" << endl;
			input = portIn.read(true);
			cout << "Training data received and processed" << endl;
		}


		targCount++;
	}


}


void torsoSaccading::LWPR_TorsoLearnerFullModel()
{
	bool existing = false;
	char c;
	cout << "Does a model already exist [y/n]" << endl;
	cin >> c;
	if(c=='y')
		existing = true;



	int targCount=0, numTargets=1, numSamples=10;


	string fullpath = params.path + "torsoLWPRlog.txt";
	string fullLogpath = params.path + "torsoFullLWPRlog.txt";
	if(existing)
	{
		LWPR_Logfile.open(fullpath.c_str(), ios::out | ios::app);	//append the contents onto the end of the file
		LWPR_FullLogfile.open(fullLogpath.c_str(), ios::out | ios::app);	//append the contents onto the end of the file
	}
	else
	{
		LWPR_Logfile.open(fullpath.c_str());
		LWPR_Logfile << "gazex gazey vergence torsoRot torsoTilt newVerg newTorRot newTorTilt"<< endl;
		LWPR_FullLogfile.open(fullLogpath.c_str());
		LWPR_FullLogfile << "gazex gazey vergence torsoRot torsoTilt desiredGazex desiredGazey desiredVerg torRelRot torRelTilt timeStamp"<< endl;
	}

	//if not got an existing model then need to initialise the matlab model.
	if(!existing)
	{
		cout << "Requesting LWPR initialisation" << endl;
		Time::delay(0.5);
		Bottle& b2 = portOut.prepare();
		b2.clear();
		b2.addInt(0);
		portOut.write();

		cout << "Waiting for confirmation of initialisation" << endl;
		input = portIn.read(true);
		existing = true;
		cout << "Confirmation received, proceeding with learning" << endl;
	}

	double targX, targY;
	string colour = "red";  //TODO make parameter

	float record[numSamples][6];

	while(targCount < numTargets)
	{

//		Request new starting position
//		send 1, followed by a desired number of training samples
		cout << "******************************************************" << endl;
		cout << "Requesting new starting point" << endl;
		double* torBabX = babbleGenerator(numSamples+1, -35, 35);
		double* torBabY = babbleGenerator(numSamples+1, -15, 30);
		for(int i=0; i<numSamples; i++)
		{
			cout << "(" << torBabX[i] << ", " << torBabY[i] << ")";
		}
		cout << endl;


		printf("Starting point received at (%.2f, %.2f)\n", torBabX[0], torBabY[0]);

		cout << "Moving to new starting point" << endl;
//		move torso to x, y
//		ehCont->toRest();
		torso->move(torBabX[0], torBabY[0], true);
		logEntry("movement ", torBabX[0], torBabY[0]);


		cout << "Target " << targCount << ": Please move the target to a new location (that I can see) " <<
				"and type 'y' when ready (x to quit, or t to test)" << endl;
		cin >> c;
//		if(c == 't')
//		{
//			LWPR_Tester(colour, eyeAndHead);
//
//			cout << "Moving back to starting point" << endl;
////			move torso to x, y
//			ehCont->toRest();
//			torso->move(rot, pitch, true);
//
//			cout << "Target " << targCount << ": Please move the target to a new location (that I can see) " <<
//					"and type 'y' when ready (x to quit)" << endl;
//			cin >> c;
//		}


		if(c != 'y')
		{
			LWPR_Logfile.close();
			LWPR_FullLogfile.close();
			return;
		}
		double depth;
		cout << "Attempting to fixate on the target" << endl;
		bool visible = target->getTarget(&targX, &targY, colour);
		if(visible)
		{
			ehCont->fixate(targX, targY, colour, false);
			ehCont->verge(colour, &depth, true);
		}
		else
		{
			continue;
		}

//		if(!fixated)
//			continue;



		cout << "Target " << targCount << ": Please move target so I'm focused on the target using only the torso" <<
				" and type 'y' when ready (or x to quit)" << endl;
//		cin >> c;
//		if(c != 'y')
//		{
//			LWPR_Logfile.close();
//			LWPR_FullLogfile.close();
//			return;
//		}


//		record starting torso position and eye vergence
		cout << "Recording starting positions" << endl;
		double* headStartingPose = new double[6];
		double* torsoStartingPose = new double[3];


		bool success = ehCont->getHeadController()->getCurrentPosition(headStartingPose);
		success &= torso->getCurrentPosition(torsoStartingPose);
		if(!success)
		{
			continue;
		}


		double sGazeX, sGazeY;
		ehCont->getGazeDirection(&sGazeX, &sGazeY);

		tracker* t = new tracker(colour);
		t->initTrack(ehCont->getMotorDriver(), target, ehCont);




		for(int i=0; i<numSamples; i++)
		{
			cout << "Sample " << i << " of target " << targCount << " requested" << endl;
			Time::delay(0.5);
//			3.	request new babble position
//				refixate eye (and head) on target
//				verge on target

			printf("Babble position (%.2f,%.2f) received\n", torBabX[i+1], torBabY[i+1]);

			boost::thread trackThrd(boost::bind(&tracker::track,t)); 	//Attempt to track the target as the torso moves


//			move torso to new position
			cout << "Moving torso to new position" << endl;
			torso->move(torBabX[i+1], torBabY[i+1], true);
			logEntry("movement ", torBabX[i+1], torBabY[i+1]);

			Time::delay(1.5);
			trackThrd.interrupt();		//stop tracking when torso has finished moving


			while(!ehCont->allStationary())
				Time::delay(0.5);


			Time::delay(0.5);
			target->getTarget(&targX, &targY, colour);


			if(!target->targetCentred())
			{
				cout << "Sample " << i << " of target " << targCount <<
						". Failed to re-fixate gaze, please could you help me [y/n]" << endl;

				c='n';
			}
			else{
				cout << "Sample " << i << " of target " << targCount << endl; //". Please confirm that I am fixated [y/n]." << endl;
				c = 'y';
				record[i][5] = 1;
			}
//			cin >> c;

			if (c != 'y')
			{
				cout << "Oh well, will try again at the next position." << endl;
				record[i][5] = -1;
				continue;
			}



			cout << "Attempting to re-verge on target" << endl;
			ehCont->verge(colour, &depth, true);
			cout << "Please make sure I am verged on the target" << endl;
//			cin >> c;

			cout << "Calculating parameters for learning" << endl;
			double* headNewPose = new double[6];
			double* torsoNewPose = new double[3];
			success = ehCont->getHeadController()->getCurrentPosition(headNewPose);
			success &= torso->getCurrentPosition(torsoNewPose);
			if(!success)
			{
				record[i][5] = -1;
			}
			else
			{
				record[i][5] = 1;
			}

			double gazeX, gazeY;
			ehCont->getGazeDirection(&gazeX, &gazeY);
			record[i][0] = gazeX;
			record[i][1] = gazeY;
			record[i][2] = headNewPose[5];

			if(isnan(gazeX)||isnan(gazeY))
			{
				record[i][5] = -1;
				continue;
			}

			double torsoRotChange, torsoPitchChange;
			torsoRotChange = torsoNewPose[0] - torsoStartingPose[0];
			torsoPitchChange = torsoNewPose[2] - torsoStartingPose[2];

			record[i][3] = torsoNewPose[0];
			record[i][4] = torsoNewPose[2];

//			4.	send training data
//				send 3, followed by current gaze direction, current vergence, current torso position, recorded vergence from starting position,
//					and difference between starting torso position and current torso position (starting - current)

			cout << "Sending training data" << endl;
			Time::delay(0.5);
			Bottle& b2 = portOut.prepare();
			b2.clear();
			b2.addInt(3);
			b2.addDouble(sGazeX);
			b2.addDouble(sGazeY);
			b2.addDouble(headStartingPose[5]);	//vergence
			b2.addDouble(torsoStartingPose[0]);	//torso rotation
			b2.addDouble(torsoStartingPose[2]);	//torso pitch
			b2.addDouble(gazeX);	//desired
			b2.addDouble(gazeY);
			b2.addDouble(headNewPose[5]);
			b2.addDouble(torsoRotChange);
			b2.addDouble(torsoPitchChange);
			portOut.write();

			time_t current = time(NULL);
			LWPR_Logfile << sGazeX << " " << sGazeY << " " << headStartingPose[5] << " " << torsoStartingPose[0] << " " << torsoStartingPose[2] << " ";
			LWPR_Logfile << headNewPose[5] << " " << torsoRotChange << " " << torsoPitchChange << endl;

			LWPR_FullLogfile << sGazeX << " " << sGazeY << " " << headStartingPose[5] << " " << torsoStartingPose[0] << " " << torsoStartingPose[2] << " ";
			LWPR_FullLogfile << gazeX << " " << gazeY << " " << headNewPose[5] << " "<< torsoRotChange << " " << torsoPitchChange  <<
						" " << current << endl;

//			receive a default message when training data has been processed
			cout << "Waiting for confirmation from LWPR training" << endl;
			input = portIn.read(true);
			cout << "Training data received and processed" << endl;


			torsoRotChange = torsoStartingPose[0]-torsoNewPose[0];
			torsoPitchChange = torsoStartingPose[2]-torsoNewPose[2];

//			4.	send training data
//				send 3, followed by current gaze direction, current vergence, current torso position, recorded vergence from starting position,
//					and difference between starting torso position and current torso position (starting - current)

			cout << "Sending training data" << endl;
			Time::delay(0.5);
			b2 = portOut.prepare();
			b2.clear();
			b2.addInt(3);
			b2.addDouble(gazeX);
			b2.addDouble(gazeY);
			b2.addDouble(headNewPose[5]);	//vergence
			b2.addDouble(torsoNewPose[0]);	//torso rotation
			b2.addDouble(torsoNewPose[2]);	//torso pitch
			b2.addDouble(sGazeX);	//desired
			b2.addDouble(sGazeY);
			b2.addDouble(headStartingPose[5]);
			b2.addDouble(torsoRotChange);
			b2.addDouble(torsoPitchChange);
			portOut.write();

			current = time(NULL);
			LWPR_Logfile << gazeX << " " << gazeY << " " << headNewPose[5] << " " << torsoNewPose[0] << " " << torsoNewPose[2] << " ";
			LWPR_Logfile << headStartingPose[5] << " " << torsoRotChange << " " << torsoPitchChange << endl;

			LWPR_FullLogfile << gazeX << " " << gazeY << " " << headNewPose[5] << " " << torsoNewPose[0] << " " << torsoNewPose[2] << " ";
			LWPR_FullLogfile << sGazeX << " " << sGazeY << " " << headStartingPose[5] << " " << torsoRotChange << " " << torsoPitchChange  <<
						" " << current << endl;

//			receive a default message when training data has been processed
			cout << "Waiting for confirmation from LWPR training" << endl;
			input = portIn.read(true);
			cout << "Training data received and processed" << endl;

		}

		cout << "Checking headRecord" << endl;
		for(int i=0; i<numSamples; i++)
		{
			for(int j=0; j<6; j++)
			{
				cout << record[i][j] << ",";
			}
			cout << endl;
		}



		cout << "Obtaining gaze map" <<endl;
		GazeMap* gm = ehCont->getGazeMap();
		cout << "Attempting to learn a whole load more movements" << endl;
		for(int i=0; i<numSamples; i++)
		{
			if(record[i][5]==-1)
				continue;

			float torsoRotChange, torsoPitchChange;

			for(int j=0; j<numSamples; j++)
			{
				if(record[j][5]==-1)
					continue;

				if(i==j)
					continue;


				torsoRotChange = record[j][3] - record[i][3];
				torsoPitchChange = record[j][4]  - record[i][4] ;

				cout << "Sending training data" << endl;
				Time::delay(0.5);
				Bottle& b2 = portOut.prepare();
				b2.clear();
				b2.addInt(3);
				b2.addDouble(record[i][0]);
				b2.addDouble(record[i][1]);
				b2.addDouble(record[i][2]);	//vergence
				b2.addDouble(record[i][3]);	//torso rotation
				b2.addDouble(record[i][4]);	//torso pitch
				b2.addDouble(record[j][0]);	//desired
				b2.addDouble(record[j][1]);
				b2.addDouble(record[j][2]);
				b2.addDouble(torsoRotChange);
				b2.addDouble(torsoPitchChange);
				portOut.write();


				LWPR_Logfile << record[i][0] << " " << record[i][1] << " " << record[i][2] << " " << record[i][3] << " " << record[i][4] << " ";
				LWPR_Logfile << record[j][2] << " " << torsoRotChange << " " << torsoPitchChange << endl;

				time_t current = time(NULL);
				LWPR_FullLogfile << record[i][0] << " " << record[i][1] << " " << record[i][2] << " " << record[i][3] << " " << record[i][4] << " ";
				LWPR_FullLogfile << record[j][0] << " " << record[j][1] << " " << record[j][2] << " " << torsoRotChange << " " << torsoPitchChange <<
						" " << current << endl;

	//			receive a default message when training data has been processed
				cout << "Waiting for confirmation from LWPR training" << endl;
				input = portIn.read(true);
				cout << "Training data received and processed" << endl;
			}
		}


		targCount++;
	}

	LWPR_Logfile.close();
	LWPR_FullLogfile.close();

}



/**
 * Continues to train from test data.
 */
void torsoSaccading::LWPR_Tester(string colour, bool eyeAndHead)
{
	double targX, targY;

	string fullpath = params.path + "torsoLWPR_testLog.txt";
	ofstream LWPR_TestLogfile;
	LWPR_TestLogfile.open(fullpath.c_str(), ios::out | ios::app);
	//"StartingPointsCompleted TestNum HS0 HS1 HS2 HS3 HS4 HS5 TS0 TS1 TS2 GazeXS GazeYS ModelVerg ModRelTorRot ModRelTorTilt HE0 HE1 HE2 HE3 HE4 HE5 TE0 TE1 TE2 DistToTargEnd"
	int startingPointCompleted;
	cout << "Please enter the number of starting points completed for the log" << endl;
	cin >> startingPointCompleted;


	int testCount = 0;
	char c;
	cout << "*********************** TORSO LWPR TESTING *************************" << endl;
	cout << "Target " << testCount << ": Please move the target to a new location (that I can see), without being directly fixated and type 'y' when ready (x to quit)" << endl;
	cin >> c;

	while(c=='y')
	{
		LWPR_TestLogfile << startingPointCompleted << " " << testCount << " ";

		//fixate on the target
		target->getTarget(&targX, &targY, colour);
		ehCont->fixate(targX, targY, colour, false);
		double depth;
		if(eyeAndHead)
			ehCont->verge(colour, &depth, true);
		else
			ehCont->verge(colour, &depth, false);
		cout << "Please make sure I am focused on the target" << endl;
		cin >> c;

		//Obtain current gaze direction
		cout << "Calculating current gaze direction" << endl;
		double* headNewPose = new double[6];
		double* torsoNewPose = new double[3];
		ehCont->getHeadController()->getCurrentPosition(headNewPose);
		torso->getCurrentPosition(torsoNewPose);

		double gazeX, gazeY;
		if(eyeAndHead)
			ehCont->getGazeDirection(&gazeX, &gazeY);
		else
		{
			gazeX = headNewPose[4];
			gazeY = headNewPose[3];
		}

		for(int i=0; i<6; i++)
		{
			LWPR_TestLogfile << headNewPose[i] << " ";
		}
		for(int i=0; i<3; i++)
		{
			LWPR_TestLogfile << torsoNewPose[i] << " ";
		}
		LWPR_TestLogfile << gazeX << " " << gazeY << " ";


		//request movement from matlab to fixate on target
		cout << "Sending: " << gazeX << ", " << gazeY << ", " << headNewPose[5] << ", "
				<< torsoNewPose[0] << ", " << torsoNewPose[2] << endl;
		Time::delay(0.5);
		Bottle& b1 = portOut.prepare();
		b1.clear();
		b1.addInt(4);		//request config, need to send inputs
		b1.addDouble(gazeX);
		b1.addDouble(gazeY);
		b1.addDouble(headNewPose[5]);
		b1.addDouble(torsoNewPose[0]);	//torso rotation
		b1.addDouble(torsoNewPose[2]);	//torso pitch
		portOut.write();

		cout << "Waiting for response from model" << endl;
		input = portIn.read(true);
		double verge, relRot, relPitch;
		verge = input->get(0).asDouble();
		relRot = input->get(1).asDouble();
		relPitch = input->get(2).asDouble();

		LWPR_TestLogfile << verge << " " << relRot << " " << relPitch << " ";

		double roll, pitch;
		roll = torsoNewPose[0] + relRot;
		pitch = torsoNewPose[2] + relPitch;

		cout << "Received vergence and relative torso command: " << verge << ", " << relRot << ", " << relPitch << endl;
		cout << "Moving torso to: " << roll << ", " << pitch << endl;

		//make movement, centering eye and head
		torso->move(roll, pitch, false);
		ehCont->toRest();
		ehCont->getEyeController()->verg(verge,true);


		//see how accurate the output was.
		double nTargX, nTargY, dist;
		target->getTarget(&nTargX, &nTargY, colour);
		target->fovea(nTargX, nTargY, &dist);
		cout << "The target is at a distance of " << dist << " from the centre." << endl;	//TODO: Should probably be recording this in a logfile
		if(!target->targetCentred())
		{
			cout << "This doesn't quite seem to have worked.  Please could you move my torso so I'm focused on the target." << endl;
			cout << "Press any key when ready." << endl;
		}
		else
		{
			cout << "This seems to have worked nicely, please confirm."  << endl;
		}
		cin >> c;

		double* headStart = new double[6];
		double* torsoStart = new double[3];
		ehCont->getHeadController()->getCurrentPosition(headStart);
		torso->getCurrentPosition(torsoStart);

		for(int i=0; i<6; i++)
		{
			LWPR_TestLogfile << headStart[i] << " ";
		}
		for(int i=0; i<3; i++)
		{
			LWPR_TestLogfile << torsoStart[i] << " ";
		}
		LWPR_TestLogfile << dist << endl;

		if(c=='y')
		{
			cout << "Sending learning data to Matlab" << endl;


			double torsoRotChange, torsoPitchChange;
			torsoRotChange = torsoNewPose[0] - torsoNewPose[0];
			torsoPitchChange = torsoNewPose[2] - torsoNewPose[2];

	//			4.	send training data
	//				send '3', followed by current gaze direction, current vergence, current torso position, recorded vergence from starting position,
	//					and difference between starting torso position and current torso position (starting - current)

			cout << "Sending training data" << endl;
			Time::delay(0.5);
			Bottle& b2 = portOut.prepare();
			b2.clear();
			b2.addInt(3);
			b2.addDouble(gazeX);
			b2.addDouble(gazeY);
			b2.addDouble(headNewPose[5]);	//vergence
			b2.addDouble(torsoNewPose[0]);	//torso rotation
			b2.addDouble(torsoNewPose[2]);	//torso pitch
			b2.addDouble(headStart[5]);	//starting vergence
			b2.addDouble(torsoRotChange);
			b2.addDouble(torsoPitchChange);
			portOut.write();

			LWPR_Logfile << gazeX << " " << gazeY << " " << headNewPose[5] << " " << torsoNewPose[0] << " " << torsoNewPose[2] << " ";
			LWPR_Logfile << headStart[5] << " " << torsoRotChange << " " << torsoPitchChange << endl;

			input = portIn.read(true);
			cout << "Training data received and processed" << endl;
		}
		testCount++;

		cout << "*********************** TORSO LWPR TESTING *************************" << endl;
		cout << "Target " << testCount << ": Please move the target to a new location (that I can see), without being directly fixated and type 'y' when ready (x to quit)" << endl;
		cin >> c;

	}
	cout << "Completed " << testCount << " tests" <<endl;
	LWPR_TestLogfile.close();
	return;
}


void torsoSaccading::LWPR_TorsoReach(double desiredGazeX, double desiredGazeY, double desiredVergence)
{
	double gazeX, gazeY, vergence, torRot, torTilt;

	//current gaze direction and vergence
	ehCont->getGazeDirection(&gazeX, &gazeY);
	ehCont->getVergence(&vergence);

	//current torso orientation
	torso->getCurrentPosition(&torRot, &torTilt);

	//request movement from matlab to fixate on target
	cout << "Sending matlab: " << gazeX << ", " << gazeY << ", " << vergence << ", "
			<< torRot << ", " << torTilt << ", "
			<< desiredGazeX << ", " << desiredGazeY << ", " << desiredVergence << endl;
	Time::delay(0.5);
	Bottle& b1 = portOut.prepare();
	b1.clear();
	b1.addInt(4);		//request config, need to send inputs
	b1.addDouble(gazeX);
	b1.addDouble(gazeY);
	b1.addDouble(vergence);
	b1.addDouble(torRot);	//torso rotation
	b1.addDouble(torTilt);	//torso pitch
	b1.addDouble(desiredGazeX);
	b1.addDouble(desiredGazeY);
	b1.addDouble(desiredVergence);
	portOut.write();

	Bottle* input = portIn.read(true);
	//returns relative torso rotation and tilt
	double relTorRot, relTorTilt;
	relTorRot = input->get(0).asDouble();
	relTorTilt = input->get(1).asDouble();
	cout << "Received from Matlab: " << relTorRot << ", " << relTorTilt << endl;
	logEntry("LWPR_rel ", relTorRot, relTorTilt);


	logEntry("LWPR_act ", torRot + relTorRot, torTilt + relTorTilt);
	bool inReach = torso->move(torRot + relTorRot, torTilt + relTorTilt, true);		//Returns true if it hasn't had to modify any of the values


}

void torsoSaccading::LWPR_TorsoReach(string colour, bool rightArm)
{
	double gazeX, gazeY, vergence, torRot, torTilt;

	//current gaze direction and vergence
	ehCont->getGazeDirection(&gazeX, &gazeY);
	ehCont->getVergence(&vergence);

	//current torso orientation
	torso->getCurrentPosition(&torRot, &torTilt);

	//which arm do you want to reach with?
	//sweet spot:
	// x = -0.3			(in front of the iCub)
	// y = 0 +/- 0.25	(+right, -left)
	// z = 0 to 0.25	(vertical)
	//converted into gaze space?
	//
	//horizontal or vertical reach?
	double desiredGazeX, desiredGazeY, desiredVergence;
	if(rightArm)
	{
		//go to right hand sweet spot (-0.28, 0.125, 0.125)
		desiredGazeX = 25.1555;
		desiredGazeY = -49.8233;
		desiredVergence = 13.9992;
	}
	else
	{
		//go to left hand sweet spot (-0.28, -0.125, 0.125)
		desiredGazeX = -27.9314;
		desiredGazeY = -58.2326;
		desiredVergence = 13.9992;
	}


	//request movement from matlab to fixate on target
	cout << "Sending matlab: " << gazeX << ", " << gazeY << ", " << vergence << ", "
			<< torRot << ", " << torTilt << ", "
			<< desiredGazeX << ", " << desiredGazeY << ", " << desiredVergence << endl;
	Time::delay(0.5);
	Bottle& b1 = portOut.prepare();
	b1.clear();
	b1.addInt(4);		//request config, need to send inputs
	b1.addDouble(gazeX);
	b1.addDouble(gazeY);
	b1.addDouble(vergence);
	b1.addDouble(torRot);	//torso rotation
	b1.addDouble(torTilt);	//torso pitch
	b1.addDouble(desiredGazeX);
	b1.addDouble(desiredGazeY);
	b1.addDouble(desiredVergence);
	portOut.write();

	input = portIn.read(true);
	//returns relative torso rotation and tilt
	double relTorRot, relTorTilt;
	relTorRot = input->get(0).asDouble();
	relTorTilt = input->get(1).asDouble();
	cout << "Received from Matlab: " << relTorRot << ", " << relTorTilt << endl;
	logEntry("LWPR_rel ", relTorRot, relTorTilt);

	//Make a movement, trying to keep the head on the target
//	tracker* t = new tracker(colour, ehCont);	//TODO: This should probably done from a higher level, so may want to do this where ever this function is called from
//	boost::thread trackThrd(boost::bind(&tracker::track,t)); 	//Attempt to track the target as the torso moves

	logEntry("LWPR_act ", torRot + relTorRot, torTilt + relTorTilt);
	bool inReach = torso->move(torRot + relTorRot, torTilt + relTorTilt, true);		//Returns true if it hasn't had to modify any of the values

//	Time::delay(1.5);
//	trackThrd.interrupt();		//stop tracking when torso has finished moving
}

void torsoSaccading::LWPR_centerTorsoOnTarget(double desiredVergence)
{
	double gazeX, gazeY, vergence, torRot, torTilt;

	//current gaze direction and vergence
	ehCont->getGazeDirection(&gazeX, &gazeY);
	ehCont->getVergence(&vergence);

	//current torso orientation
	torso->getCurrentPosition(&torRot, &torTilt);

	double desiredGazeX=0, desiredGazeY=0;

	cout << "Sending matlab: " << gazeX << ", " << gazeY << ", " << vergence << ", "
			<< torRot << ", " << torTilt << ", "
			<< desiredGazeX << ", " << desiredGazeY << ", " << desiredVergence << endl;
	Time::delay(0.5);
	Bottle& b1 = portOut.prepare();
	b1.clear();
	b1.addInt(4);		//request config, need to send inputs
	b1.addDouble(gazeX);
	b1.addDouble(gazeY);
	b1.addDouble(vergence);
	b1.addDouble(torRot);	//torso rotation
	b1.addDouble(torTilt);	//torso pitch
	b1.addDouble(desiredGazeX);
	b1.addDouble(desiredGazeY);
	b1.addDouble(desiredVergence);
	portOut.write();

	input = portIn.read(true);
	//returns relative torso rotation and tilt
	double relTorRot, relTorTilt;
	relTorRot = input->get(0).asDouble();
	relTorTilt = input->get(1).asDouble();
	cout << "Received from Matlab: " << relTorRot << ", " << relTorTilt << endl;

	bool inReach = torso->move(torRot + relTorRot, torTilt + relTorTilt, false);
	ehCont->toRest();
	ehCont->getEyeController()->verg(desiredVergence, false);

}


void torsoSaccading::closeMatlabPorts()
{
	Time::delay(0.5);
	Bottle& b1 = portOut.prepare();
	b1.clear();
	b1.addInt(5);		//quit
	portOut.write();

	portOut.close();
	portIn.close();

//	LWPR_Logfile.close();
}







/*****************************************
 *  LOGGING FUNCTIONS:
 *****************************************
 */


void torsoSaccading::openLogs()
{
	string fullpath = params.path + "torsomotorlog.txt";
//	motorlogfile.open(fullpath.c_str());

	fullpath = params.path + "torsolog.txt";
	logfile.open(fullpath.c_str());


//	motorlogfile << "x y saccadeNo"<< endl;
}


void torsoSaccading::motorRecord(double relX, double relY, int saccadeCounter)
{
	motorlogfile << relX << " " << relY << " "
						<< saccadeCounter << " " << endl;
}

void torsoSaccading::logEntry(string message)
{
	cout << message << endl;
	logfile << message << endl;
}

void torsoSaccading::logEntry(string message, double x, double y)
{
	cout << message << x << ", " << y << endl;
	logfile << message << x << ", " << y << endl;
}

void torsoSaccading::logField(const Field* field)
{
	cout << *field << endl;
	logfile << *field << endl;
}

void torsoSaccading::startSaccadeLog(int psaccadeCounter)
{
	saccadeCounter = psaccadeCounter;
	motorlogfile << "*******************************" << endl;
	motorlogfile << "Starting saccade: " << saccadeCounter << endl;

	logfile << "*******************************" << endl;
	logfile << "Starting saccade: " << saccadeCounter << endl;

}
void torsoSaccading::endSaccadeLog(bool success)
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

void torsoSaccading::closeLogs()
{
	logfile << "**************END**************" << endl;
	logfile.close();

	motorlogfile << "**************END**************" << endl;
	motorlogfile.close();
}



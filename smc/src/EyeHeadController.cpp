/*
 * EyeHeadController.cpp
 *
 *  Created on: 8 Jun 2011
 *      Author: icub
 */



#include "EyeHeadController.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;




EyeHeadSaccading::EyeHeadSaccading(GazeMap* pGM, Target* ptarget)
{
	target = ptarget;

	initYarp();
	init();

	gm = pGM;


}


/**
 * This is currently only used by the BBcontroller, as the gazemap is not
 * necessary.  If wanting to use this later, will have to pass the gazemap
 * back, and link it in elsewhere.
 */
EyeHeadSaccading::EyeHeadSaccading(Target* ptarget)
{
	target = ptarget;

	initYarp();
	init();

	gm = new GazeMap();	//TODO this will be lost unless passed back

}


EyeHeadSaccading::~EyeHeadSaccading()
{

	delete head_ppm;
	delete eye_ppm;
	motordriver->close();
}


void EyeHeadSaccading::init()
{
	initMaps();
	initLogs();

	eyeCont = new eyeController(motordriver);
	eyeSac = new eyeSaccading(eyeCont, target, eye_ppm);

	cout << "Eye controllers initialised" << endl;

	headCont = new headController(motordriver);
	headSac = new headSaccading(headCont, eyeSac, target, head_ppm);

	cout << "Head controllers initialised" << endl;

	v = new vor();
	v->initVor(motordriver);

	cout << "Vor initialised" << endl;

	eyeOnlyCounter = 0;	//eye saccades
	saccadeCounter = 0;

	eyeOnlyDuration = 30*60;
	headDuration = 30*60;
	useThresholds = true;

	rollingAverage = 100;
	eyeThreshold = 2.0;
	combiSuccessful = 0;
	combiClose = 0;
	combiFailed = 0;
	eyeOnly = 0;		//combi eye+head saccades that only actually require the eye
	eyeHeadSuccessRate = 0;

	eyeHeadThreshold = 0.75;	//75%
	combiThreshold = 5;


}


bool EyeHeadSaccading::initYarp()
{
	yarp::os::Network yarp;

	//Set up yarp network to receive target data and send motor commands:
	Property options;
	options.put("device", "remote_controlboard");
	string hport = "/headmove";
	if(params.m_ROBOT.compare("icubSim")==0)
		hport += "Sim";
	options.put("local", hport.c_str());
	string head = "/";
	head += params.m_ROBOT;
	head += "/head";
	//options.put("remote", "/icub/head");
	options.put("remote", head.c_str());


	motordriver=new PolyDriver;
	motordriver->open(options);


	if (!motordriver->isValid())
	{
		cout << "Failed to open device\n";
		return false;
	}

	return true;
}

bool EyeHeadSaccading::allStationary()
{
	return headSac->allStationary();
}
void EyeHeadSaccading::toRest()
{
	eyeCont->toRest();
	headCont->toRest();
	eyeCont->verg(0, false);
}

void EyeHeadSaccading::babble()
{
	eyeCont->babble();
	headCont->babble();
}

void EyeHeadSaccading::smallBabble(double maxRange)
{
	eyeCont->smallbabble(true, maxRange);
	headCont->smallbabble(true, maxRange);
}

bool EyeHeadSaccading::initMaps()
{
//TODO: Adjust this to use parameters of retina dimensions from params_config.h
		eye_ppm = new ffm(INPUT_MAP_TYPE,  0.f, RETINA_WIDTH,   0.f, RETINA_HEIGHT,
						  OUTPUT_MAP_TYPE,  -60.f,  60.f, -53.f,  53.f);
							//relative motor values

	//Head vision now in eye-head gaze space, y should have max 360, but need
	//to keep fovea in same location as in retina map.
		head_ppm = new ffm(INPUT_MAP_TYPE, -1*RETINA_WIDTH, 2*RETINA_WIDTH, -1*RETINA_HEIGHT, 2*RETINA_HEIGHT,
					       OUTPUT_MAP_TYPE,   -101.f, 101.f,  -56.f,  56.f);
								//relative motor values
//	size_t numFields;
//	head_ppm->setInputFields(eye_ppm->getAllInputFields(&numFields));

//	gm = new GazeMap();		// gaze map needs to be passed down from higher level as linked to reach

	if(params.m_LOAD)
		loadFile(params.m_FILENAME);


	return true;
}

bool EyeHeadSaccading::reloadMaps(string filename)
{
	bool success = loadFile(filename);
	if(success)
	{
		eyeSac->reloadMaps(eye_ppm);
		headSac->reloadMaps(head_ppm);
	}
	return success;
}


bool EyeHeadSaccading::loadFile(string filename)
{

	bool success = false;

		FFM_IO io;

		try{
		delete eye_ppm;
		eye_ppm = io.loadMappingFromXML(params.m_PATH + "eye_" + filename +".xml");
//		eye_ppm->printLinkedFields();
		cout << "There are " << eye_ppm->getNumLinks() << " links in the eye map"<< endl;
		}
		catch(const IMapException & ime)
		{
			cout << "Error trying to load eye mappings, generating blank mapping" << endl;
				eye_ppm = new ffm(INPUT_MAP_TYPE,  0.f, RETINA_WIDTH,   0.f, RETINA_HEIGHT,
								  OUTPUT_MAP_TYPE,  -60.f,  60.f, -53.f,  53.f);
									//relative motor values
		}

		try{
		delete head_ppm;
		head_ppm = io.loadMappingFromXML(params.m_PATH + "head_" + filename +".xml");
		cout << "There are " << head_ppm->getNumLinks() << " links in the head map"<< endl;
		}
		catch(const IMapException & ime)
		{
			cout << "Error trying to load head mappings, generating blank mapping" << endl;
				head_ppm = new ffm(INPUT_MAP_TYPE, -1*RETINA_WIDTH, 2*RETINA_WIDTH, -1*RETINA_HEIGHT, 2*RETINA_HEIGHT,
								   OUTPUT_MAP_TYPE,   -101.f, 101.f,  -56.f,  56.f);
		}

//		Gaze_IO gm_io;
//		delete gm;
//		gm = gm_io.loadMappingFromXML(path + "GM_" + filename);
//		cout << "There are " << gm->getNumFields() << " fields in the gaze map" << endl;

//		head_ppm->printLinkedFields();

//		size_t numFields;
//		head_ppm->setInputFields(eye_ppm->getAllInputFields(&numFields));

		cout << "Loaded links from existing file: " << filename << endl;


//		cout << "Would you like to enable learning? y/n" << endl;
//		cin >> in;
//		if(in == 'y')
//		{
//			learn = true;
//			cout << "Learning has been enabled" << endl;
//		}
//		else
//		{
//			learn = false;
//			cout << "Learning has been disabled" << endl;
//		}

		success = true;
//	}



	return success;
}

bool EyeHeadSaccading::saveMaps()
{
	bool success = true;
	FFM_IO io;
	try{
		io.saveMappingToXML(eye_ppm,params.m_PATH + "eye_" + params.m_FILENAME +".xml");
		cout << eye_ppm->getNumLinks() << " Eye links successfully saved"<<endl;
	}
	catch(const IMapException & ime)
	{
		cout << "An error occurred whilst attempting to save the eye links"<<endl;
		success = false;
	}
//	if(io.saveMappingToXML(eye_ppm,path + "eye_" + filename +".xml"))
//		cout << eye_ppm->getNumLinks() << " Eye links successfully saved"<<endl;
//	else
//		cout << "An error occurred whilst attempting to save the eye links"<<endl;

	FFM_IO io2;
	try{
		io2.saveMappingToXML(head_ppm,params.m_PATH + "head_" + params.m_FILENAME+".xml");
		cout << head_ppm->getNumLinks() <<  " Head links successfully saved"<<endl;
	}
	catch(const IMapException & ime)
	{
		cout << "An error occurred whilst attempting to save the head links"<<endl;
		success = false;
	}
//	if(io2.saveMappingToXML(head_ppm,path + "head_" + filename+".xml"))
//			cout << head_ppm->getNumLinks() <<  " Head links successfully saved"<<endl;
//	else
//		cout << "An error occurred whilst attempting to save the head links"<<endl;

	return success;
}


void EyeHeadSaccading::initLogs()
{
	string fullpath = params.m_PATH + "eye_stats.txt";
	eyestatslog.open(fullpath.c_str());
	eyestatslog << "saccadeCounter stepCounter successfulDirectLink successfulNeighbourLink " <<
			"unsuccesfulDirectLinkCounter neighourCounter penultimateMoveWasLink " <<
			"linksLearnt linksUpdated possibleLinkstoLearn unlearntInputFields("
			<< eye_ppm->getNumInputFields() << ") unlearntOutputFields("
			<< eye_ppm->getNumOutputFields() << ") rollingAverage NumLinks NumGoodLinks TimeStamp" << endl;

	time_t timeStamp;
	timeStamp = time(NULL);
	eyestatslog << "-" << " " << "-" << " " << "-" << " "
			<< "-" << " " << "-" << " "
			<< "-" << " " << "-" << " " << "-" << " "
			<< "-" << " " << "-" << " " << "-" << " "
			<< "-" << " " << "-" << " " << "-" << " " << "-" <<
			" " << timeStamp << endl;


	fullpath = params.m_PATH + "head_stats.txt";
	headstatslog.open(fullpath.c_str());
	headstatslog << "saccadeCounter combieyeonly combisuccessful combiclose combifailed " <<
			"unlearntInputFields(" << head_ppm->getNumInputFields()
			<<") unlearntOutputFields(" << head_ppm->getNumOutputFields()
			<<") combisuccessrate NumLinks NumGoodLinks TimeStamp" << endl;



	headstatslog << "-" << " " << "-" << " "<< "-" << " "<< "-" << " "<< "-"
				<< " " << "-" << " " << "-" << " " << "-" << " "
				<< "-" << " " << "-" << " " << timeStamp << endl;


	fullpath = params.m_PATH +"eye+head.txt";
	eyeHeadlog.open(fullpath.c_str());

	fullpath = params.m_PATH + "headLinkLog.txt";
	headLinkLog.open(fullpath.c_str());
	headLinkLog << "retX retY motX motY time" << endl;

}


int EyeHeadSaccading::learnEyeSaccades()
{

	time_t startSeconds;
	startSeconds = time(NULL);
	int counter = 0;
	time_t timeTaken = startSeconds-startSeconds;

//	while(timeTaken < eyeOnlyDuration)	//learn for 1hr
//	while(rollingAverage > eyeThreshold)
	while((useThresholds && (rollingAverage > eyeThreshold)) || (!useThresholds && (timeTaken < eyeOnlyDuration)))
	{
		bool success = eyeSaccade();
		if(success)
		{
			counter++;

		}
		Time::delay(0.5);
		time_t current = time(NULL);
		timeTaken = current - startSeconds;
	}

	time_t endSeconds;
	endSeconds = time(NULL);
	timeTaken = endSeconds-startSeconds;
//	eyeSac->stopLearning();
	cout << "Eye saccade learning took " << timeTaken << " seconds, and required " << counter
			<< " saccades to reach eye threshold" << endl;
	eyeHeadlog << "Required " << counter << " saccades to reach eye threshold" << endl;
	eyeHeadlog << "Eye saccade learning took " << timeTaken << " seconds" << endl;
	return counter;
}


bool EyeHeadSaccading::fixate(double targX, double targY, string colour, bool simple)
{
	double dist;
	if(target->fovea(targX, targY, &dist))
		return true;

	bool success = checkCombiLinks(targX, targY, colour);
	if(!success)
	{
		if(simple)
			success = eyeSac->simpleSaccade(targX, targY, colour);
		else
		{
			saccadeCounter++;
			success = eyeSac->saccade(saccadeCounter,targX, targY, colour);
		}

	}

	if(!success && !simple)
	{
//		centreEyesInHead(); //TODO, make this check if using head yet or not!!
		saccadeCounter++;
		success = eyeSac->saccade(saccadeCounter,targX, targY, colour);
	}
	if(success)
		addGazeField();
	return success;
}


bool EyeHeadSaccading::fixate(int xl, int yl, int xr, int yr)
{
	double dist;
	bool success= target->fovea(xr,yr,&dist);
	if(dist > 8)
	{
		success = checkCombiLinks(xr, yr, "", false);	//Cannot supply a colour to check if this worked.
		if(!success)// assume no matching head link found, try with just an eye saccade
		{
			success = eyeSac->simpleSaccade(xr, yr, "", false);
		}
		else
		{
			eyeSac->fr.combiSaccade = true;
		}
	}
	else
	{
		success = eyeSac->autoCenter(xr, yr, "", false);
	}

	// Attempt to verge
	int diff = xr - xl;
	if(diff>-0.5 && diff <0.5)
	{
		cout << "Target is in focus" << endl;
	}

	else if(diff>0)
	{ //If eyes are already verged, can diverge the eyes to fixate, without resetting each time...
		cout << "Target is divergent or mis-matched: r-l="<< diff <<endl;
		double current;
		double adjustment = 1.5;
		eyeCont->getVergence(&current);
		if(current > 1)
		{
			adjustment = -2;
			if (diff < 1)
				adjustment = -0.5;
			else if (diff < 2.5)
				adjustment = -0.75;
			else if (diff < 4.5)
				adjustment = -1;

			eyeCont->verg(current+adjustment,true);
		}
	}
	else
	{
		//Meaningful difference is -1 --> -6, beyond this, it is likely that the object is not recognised equally in both eyes
		cout << "Starting to focus on target: r-l="<<diff << endl;
		double current;
		double adjustment = 1.5;
		adjustment = 2;
		if (diff > -1)
			adjustment = 0.5;
		else if (diff > -2.5)
			adjustment = 0.75;
		else if (diff > -4.5)
			adjustment = 1;

		eyeCont->verg(current+adjustment,true);
	}


	return success;
}

//Fixate on something
bool EyeHeadSaccading::fixate()
{
	double targX, targY;
	bool success = target->getTarget(&targX, &targY);
	if(success)
	{
		string colour;
		colour = target->getColour();
		return fixate(targX, targY, colour, false);
	}
	else
	{
		cout << "No target visible" <<endl;
		eyeCont->babble();
		return false;
	}
}

//fixate on a target of colour...
bool EyeHeadSaccading::fixate(std::string colour)
{
	double targX, targY;
	bool success = target->getTarget(&targX, &targY, colour);
	if(success)
		return fixate(targX, targY, colour, false);
	else
		return false;
}


bool EyeHeadSaccading::fixateEyeHead(int xr, int yr)
{
	PolarField* eyeInput;
	PolarField* headInput;
	PolarField* eyeOutput;
	PolarField* headOutput;

	float eyeDist, headDist;

	eyeInput = eyeSac->getNearestLearntInput(xr, yr, &eyeDist);
	headInput = headSac->getNearestLearntInput(xr, yr, &headDist);

	eyeOutput = eyeSac->getLinkedOutput(eyeInput);
	headOutput = headSac->getLinkedOutput(headInput);

	float headX, headY, e_headX, e_headY, eyeX, eyeY;
	headX = headOutput->getXcoord();
	headY = headOutput->getYcoord();

	double* hmp = new double[6];
	headCont->getCurrentPosition(hmp);
//	headX += hmp[2];	//Convert relative to actual movement
//	headY += hmp[0];

	e_headX = headXtoEyeX(headX) *-1;
	e_headY = headYtoEyeY(headY) *-1;

	eyeX = eyeOutput->getXcoord();
	eyeY = eyeOutput->getYcoord();

//	eyeX += hmp[4];
//	eyeY += hmp[3];

//	if(eyeX>eyeCont->xMax)
//		eyeX = eyeCont->xMax;
//	else if(eyeX<eyeCont->xMin)
//		eyeX = eyeCont->xMin;
//
//	if(eyeY>eyeCont->yMax)
//		eyeY = eyeCont->yMax;
//	else if(eyeY<eyeCont->yMin)
//		eyeY = eyeCont->yMin;

	v->resetOverflow();
	cout << "Sending overflow: " << e_headX-eyeX << ", " << e_headY-eyeY << endl;
	v->addOverflow(e_headX-eyeX, e_headY-eyeY);

	printf("Eye movement: %.2f, %.2f\n", eyeX, eyeY);
	printf("Head movement: %.2f, %.2f\n", headX, headY);
	printf("e_Head movement: %.2f, %.2f\n", e_headX, e_headY);

	eyeSac->gotoField(eyeOutput);
	boost::thread vorThrd(boost::bind(&vor::track,v));
	headSac->gotoField(headOutput);
	Time::delay(0.2);
	vorThrd.interrupt();
	Time::delay(0.3);

	headCont->getCurrentPosition(hmp);
	eyeCont->move(hmp[4]/2,hmp[3]/2);
	//Head should be focused on target with eye centred;
	return true;


}


bool EyeHeadSaccading::eyeSaccade(){	//extract from main method on motorControllerV4.cpp
	cout << "*************Starting new eye only saccade " << eyeOnlyCounter << "**************" << endl;
	target->logSaccadeBreak(eyeOnlyCounter);
	double targX, targY;

	if(RANDOM_MOVES)
	{
		//Move eyes and head to an initial starting point
		cout << "Making a random movement to new starting position" << endl;
//		/*****
//		 *
//		 */
//		headCont->babble(false);	//sequential coverage test
//		/*****
//		 *
//		 */
		eyeCont->babble();
	}
	else
	{
		cout << "Allowing time for the target to move" << endl;
		Time::delay(5.0);
		cout << "Proceeding" << endl;
	}

	while(!(headSac->allStationary()))
	{
		Time::delay(0.2);
	}

	if(!target->getTarget(&targX, &targY))
	{
		cout << "Target not visible from here" << endl;
		return false;
	}
//		double startTargX = targX;
//		double startTargY = targY;
//		string startColour = target->getColour();


	bool success = eyeSac->saccade(eyeOnlyCounter, targX, targY, target->getColour());
	if(!success)
		return false;	// returned null input field from eye starting point or target unreachable

	double depth;
	eyeSac->getDepth(target->getColour(),&depth);
	printf("Target fixated at a depth of %.2f\n",depth);

	eyeOnlyCounter++;
	eyeStats();


	if(params.LEARN)
	{
		addGazeField();
	}

//	/******
//	 * 	Sequential coverage experiment only:
//	 */
//	v->resetOverflow();
//		//activate vor thread
////	boost::thread vorThrd(*v);
//	boost::thread vorThrd(boost::bind(&vor::track,v));
//	headCont->babble();
//	vorThrd.interrupt();
//	/******
//	 *
//	 */

	Time::delay(1);
//	int colourCheckCounter = 0;
	return true;
}


int EyeHeadSaccading::learnEyeHeadSaccades(int maximum)
{
	time_t startSeconds;
	startSeconds = time(NULL);

	int counter = 0;
	eyeHeadSuccessRate = 0;

	int prev=0;

	time_t timeTaken = startSeconds - startSeconds;
	/**********************************************************************************************
	 *  Testing long duration learning.
	 */
	while((eyeHeadSuccessRate<eyeHeadThreshold || combiSuccessful<combiThreshold)
			&& counter < maximum)
//	while(timeTaken < headDuration)	//learn for 1hr
	{
		 //PHS ALTERNATE HEADLEARNING
		if(!SYNCHRONOUS)
		{
//			eyeSac->startLearning();
			while(rollingAverage>eyeThreshold)
			{
				bool success = eyeSaccade();
				if(!success && RANDOM_MOVES)	//target may be out of reach so move head to new position
					headCont->babble();
			}
//			eyeSac->stopLearning();
		}

		if(combiSuccessful==prev)
			AltEyeHeadSaccade();
		else
			prev = combiSuccessful;
		//END PHS alternate

		bool success = eyeHeadSaccade();

		if(success)
			counter ++;

		if(combiSuccessful>0)
		{
			eyeHeadSuccessRate = (double)(combiSuccessful+combiClose) / (double)(combiSuccessful + combiClose + combiFailed);
		}
		cout << "Combi success rate: " << eyeHeadSuccessRate << endl;
		cout << "Combi successful: " << combiSuccessful << endl;

//		if(eyeHeadSuccessRate>eyeHeadThreshold)
//			break;
		Time::delay(0.5);
		time_t current = time(NULL);
		timeTaken=current - startSeconds;
	}

	time_t endSeconds;
	endSeconds = time(NULL);
	timeTaken = endSeconds-startSeconds;
	cout << "Head saccade learning took " << timeTaken << " seconds" << endl;
	eyeHeadlog << "Required " << counter << " saccades to reach head threshold" << endl;
	eyeHeadlog << "Head saccade learning took " << timeTaken << " seconds" << endl;

	return counter;
}

bool EyeHeadSaccading::eyeHeadSaccade(){		//extract of main method from motorControllerV4.cpp

	cout << "*************Starting new eye+head saccade " << saccadeCounter << "**************" << endl;
	eyeHeadlog << "*************Starting new eye+head saccade " << saccadeCounter << "**************" << endl;
	target->logSaccadeBreak(saccadeCounter);
	headSac->startSaccadeLog(saccadeCounter);
	double targX, targY;
	if(RANDOM_MOVES)
	{
		//Move eyes and head to an initial starting point
		cout << "Making a random movement to new starting position" << endl;
		eyeHeadlog << "Making a random movement to new starting position" << endl;
		eyeCont->babble(false);
		headCont->babble();//blocks till movement complete
	}
	else
	{
		cout << "Allowing time for the target to move" << endl;
		Time::delay(5.0);
		cout << "Proceeding" << endl;
	}

	while(!(headSac->allStationary()))
	{
		Time::delay(0.2);
	}
	Time::delay(0.4);

	if(!target->getTarget(&targX, &targY))
	{
		cout << "Target not visible from here" << endl;
		eyeHeadlog << "Target not visible from here" << endl;
		headSac->endSaccadeLog(false);
		return false;
	}

	double startTargX = targX;
	double startTargY = targY;
	string startColour = target->getColour();

	//looks for input fields in both eye and head
//			if(!checkInputFields(targX, targY))
//				continue;

	bool success;
	//Check whether links already exist in eye and head maps.
	if(checkCombiLinks(targX, targY, startColour))
	{

		cout << "Successfully reused combined eye and head links for saccade " << saccadeCounter << endl;
		eyeHeadlog << "Successfully reused combined eye and head links for saccade " << saccadeCounter << endl;

		eyeStats(true);
		headStats();

		eyeSac->incrementSaccadeCounter();
		saccadeCounter ++;
		Time::delay(1);
//				continue;
		success = true;

	}
	else
	{

		//move eyes to target
		headSac->recordPrePositions();	//Used to calculate the head movement
//		if(synchronous && rollingAverage>eyeThreshold)
//			eyeSac->startLearning();
//		else if(synchronous && rollingAverage<eyeThreshold)
//			eyeSac->stopLearning();

		success = eyeSac->saccade(saccadeCounter, targX, targY, startColour);
		autoCenter(startColour);
//			int eyeSteps = e->getStepCounter();

		if(!success)
		{
			Time::delay(0.2);
			if(!target->targetCentred(&targX, &targY, startColour))
			{
				eyeHeadlog << "Eye saccade failed to fixate" << endl;
				headSac->endSaccadeLog(false);
				return false;	// eye failed to fixate
			}
		}


		v->resetOverflow();
		//activate vor thread
//		boost::thread vorThrd(*v);
		boost::thread vorThrd(boost::bind(&vor::track,v));


		//move head
//		target->getTarget(&targX, &targY);	//Target should be central
		success = headSac->saccade(saccadeCounter, startTargX, startTargY, startColour, v); //learn a head mappings

		if(success)
		{
			eyeHeadlog << "Head saccade returned successfully" << endl;
		}
		else
		{
			eyeHeadlog << "Head saccade returned false" << endl;
		}

		vorThrd.interrupt();

		eyeStats();
		headStats();

		if(params.LEARN && target->targetCentred(&targX, &targY, startColour))
		{
			addGazeField();
		}

		saccadeCounter ++;
		Time::delay(1);
	}
	headSac->endSaccadeLog(success);
	return success;
}



bool EyeHeadSaccading::AltEyeHeadSaccade()
{
	int numSamples = 15;
	//Movement ranges, copied from headController.h
	const double xMin = -26.0;
	const double xMax =  26.0;
	const double yMin = -25.0;
	const double yMax =  14.0;

	double* xBabble = babbleGenerator(numSamples, xMin/2, xMax/2);
	double* yBabble = babbleGenerator(numSamples, yMin/2, yMax/2);
	cout << "****************************************" << endl;
	cout << "Babble coordinates set:" << endl;
	cout << "[";
	for(int i=0; i<numSamples; i++)
	{
		cout << "(" << xBabble[i]<< ", " <<  yBabble[i] << ")" ;
	}
	cout << "]" << endl;

	headCont->move(xBabble[0], yBabble[0], true);
	double targX, targY, dist;
	eyeCont->toRest();
	Time::delay(0.2);
	target->getNearestObject(&targX, &targY, &dist);
	string colour = target->getColour();
	Time::delay(0.2);
	eyeSac->simpleSaccade(targX, targY, colour);
	Time::delay(0.2);
	bool success = target->targetCentred(&targX, &targY, colour);
	if(!success)
		return false;



	list<altHeadChainLink> chainLinks;

	//Work through each babble pair, whilst eye remains fixated on the target
	for(int i=1; i<numSamples; i++)
	{
		cout << "Babble step " << i << endl;
		//Record starting eye+head positions
		double ex1, ey1, hx1, hy1;
		eyeCont->getCurrentPosition(&ex1, &ey1);
		headCont->getCurrentPosition(&hx1, &hy1);
		altHeadChainLink hcl(ex1, ey1, hx1, hy1);

		v->resetOverflow();
//		activate vor thread
		boost::thread vorThrd(boost::bind(&vor::track,v));
		headCont->move(xBabble[i], yBabble[i], true);

		vorThrd.interrupt();
		Time::delay(0.5);

		//Check to see if still fixated on the target
		bool centered = target->targetCentred(&targX, &targY, colour);
		Time::delay(0.2);
		bool autod = eyeSac->autoCenter(targX, targY, colour, true);
		success = centered || autod;
		if(success)
		{
			double ex2, ey2, hx2, hy2;
			eyeCont->getCurrentPosition(&ex2, &ey2);
			headCont->getCurrentPosition(&hx2, &hy2);
			hcl.update(ex2, ey2, hx2, hy2);
			chainLinks.push_back(hcl);
		}
		else
		{
			cout << "Lost babble target" << endl;
			Time::delay(0.5);
		}
	}


	//Completed a series of steps, now time to learn the links
	// Want to try and loop through so link each step against each other.
	list<altHeadChainLink>::iterator it1;
	list<altHeadChainLink>::iterator it2;
	int count =0;
	cout << "Attempting to learn head chain" << endl;
	for(it1=chainLinks.begin(); it1!=chainLinks.end(); it1++)
	{
		//Deal with single link first
		altHeadChainLink cl = *it1;
		double eyeCompX = cl.eyeX2-cl.eyeX1;
		double eyeCompY = cl.eyeY2-cl.eyeY1;
		double headBabX = cl.headX2-cl.headX1;
		double headBabY = cl.headY2-cl.headY1;

		float dist=0;
		PolarField* eyeMotor = new PolarField();
		eyeMotor = eyeSac->getNearestLearntOutput(eyeCompX, eyeCompY, &dist);
		if(dist<(eyeMotor->getRadius()*3.5) && dist>0)
		{
			PolarField* eyeLink = new PolarField();
			eyeLink = eyeSac->getLinkedInput(eyeMotor);

			FieldLink* link = eyeSac->getLink(eyeLink, eyeMotor);
			if(link->getUsage()>=1)
			{
				double targetHeadX = eyeLink->getXcoord();
				double targetHeadY = eyeLink->getYcoord();
				bool added = headSac->makeLink(targetHeadX, targetHeadY, 160, 120, -headBabX, -headBabY);
				if(added)
				{
					count++;
					headLinkLog << targetHeadX << " " << targetHeadY << " " << -headBabX << " " << -headBabY << " ";
					time_t current = time(NULL);
					headLinkLog << current << endl;

				}
			}
		}

		//Deal with two links
		int patch=0;
		for(it2=it1; it2!=chainLinks.end(); it2++)
		{
			if(patch == 0)
			{
				patch ++;
				continue;
			}
			altHeadChainLink cl2 = *it2;
			eyeCompX = cl2.eyeX2 - cl.eyeX1;
			eyeCompY = cl2.eyeY2 - cl.eyeY1;
			headBabX = cl2.headX2 - cl.headX1;
			headBabY = cl2.headY2 - cl.headY1;

			dist=0;
			PolarField* eyeMotor = new PolarField();
			eyeMotor = eyeSac->getNearestLearntOutput(eyeCompX, eyeCompY, &dist);
			if(dist<(eyeMotor->getRadius()*3.5) && dist>0)
			{
				PolarField* eyeLink = new PolarField();
				eyeLink = eyeSac->getLinkedInput(eyeMotor);

				FieldLink* link = eyeSac->getLink(eyeLink, eyeMotor);
				if(link->getUsage()>=1)
				{
					double targetHeadX = eyeLink->getXcoord();
					double targetHeadY = eyeLink->getYcoord();
					bool added = headSac->makeLink(targetHeadX, targetHeadY, 160, 120, -headBabX, -headBabY);
					if(added)
						count++;
				}
			}
		}

	}

	cout << "Added " << count << " head links" << endl;
	return true;
}


bool EyeHeadSaccading::learn_iStyleHeadLinks()
{
	eyeCont->toRest();

	time_t startSeconds;
	time_t endSeconds;
	startSeconds = time(NULL);
	int counter = 0;
	time_t timeTaken = startSeconds-startSeconds;
	while(timeTaken < eyeOnlyDuration)	//learn for 1hr
//	while(rollingAverage > eyeThreshold && counter < maximum)
	{
		cout << "*************Starting new naughty head only saccade**************" << endl;
		double targX, targY;

		if(RANDOM_MOVES)
		{
			//Move eyes and head to an initial starting point
			cout << "Making a random movement to new starting position" << endl;
			headCont->babble();
		}
		else
		{
			cout << "Allowing time for the target to move" << endl;
			Time::delay(5.0);
			cout << "Proceeding" << endl;
		}

		while(!(headSac->allStationary()))
		{
			Time::delay(0.2);
		}

		if(!target->getTarget(&targX, &targY))
		{
			cout << "Target not visible from here" << endl;
			continue;
		}

		bool success = headSac->iStyleSaccade(counter, targX, targY, target->getColour());
		if(!success)
			continue;	// returned null input field from eye starting point or target unreachable

		counter ++;


		if(params.LEARN)
		{
			addGazeField();
		}

		endSeconds = time(NULL);
		timeTaken = endSeconds-startSeconds;
		Time::delay(0.5);
	}


	cout << "Naughty head saccade learning took " << timeTaken << " seconds, and completed " << counter
			<< " saccades" << endl;
	return true;
}


void EyeHeadSaccading::learnOuterHeadLinks()
{
	double startTargX, startTargY;
	double targX, targY;
	float dist;
	string colour;
	PolarField* eyeInput1;
	PolarField* eyeOutput1;
	PolarField* eyeInput2;
	PolarField* eyeOutput2;
	PolarField* headInput;
	PolarField* headOutput;

	double* headStartConf = new double[6];
	double* headMidConf = new double[6];	//Only eye moved
	double* headEndConf = new double[6];	//Head moved to compensate eye, and eye moved again

	centreEyesInHead();

	for(int i=0; i<200; i++)
	{
		eyeCont->babble(false);
		headCont->babble();//blocks till movement complete

		target->getTarget(&targX, &targY);
		startTargX = targX;
		startTargY = targY;
		colour = target->getColour();
		headCont->getCurrentPosition(headStartConf);

		eyeInput1 = eyeSac->getNearestLearntInput(targX, targY, &dist);
		eyeOutput1 = eyeSac->getLinkedOutput(eyeInput1);

		headInput = headSac->getRetinaField(startTargX, startTargY);

		eyeSac->gotoField(eyeOutput1);

		headCont->getCurrentPosition(headMidConf);	//Only eye moved
		target->getTarget(&targX, &targY, colour);
		if(target->targetCentred())
		{
			//learn a head link
			cout << "Learning head link from single movement" << endl;
			//eyes may not have started centred
			//head may not have been able to fully centre eyes after movement

			double relEyeX = headMidConf[4] - headStartConf[4];
			double relEyeY = headMidConf[3] - headStartConf[3];

			double relHeadX = eyeXtoHeadX(relEyeX);
			double relHeadY = eyeYtoHeadY(relEyeY);

			if(headSac->isMotorField(relHeadX, relHeadY))
			{
				headInput = headSac->getMotorField(relHeadX, relHeadY);
				cout << "Making a head link between: " << *headInput << " and " << *headInput << endl;
				headSac->makeLink(headInput, headInput);
			}
			continue;
		}

		centreEyesInHead();
		target->getTarget(&targX, &targY, colour);

		eyeInput2 = eyeSac->getNearestLearntInput(targX, targY, &dist);
		eyeOutput2 = eyeSac->getLinkedOutput(eyeInput2);
		eyeSac->gotoField(eyeOutput2);	//now fixated?



		target->getTarget(&targX, &targY, colour);
		if(!target->targetCentred())
		{
			cout << "Not fixated on target after second step, going to try full saccade" << endl;
			bool success = fixate(colour);
			if(!success)
			{
				cout << "Nope, that didn't work either, giving up" << endl;
				continue;
			}
		}

		//Learn a head link
		headCont->getCurrentPosition(headEndConf);
		cout << "*** Going to try and learn a mega head link ***" << endl;

		double relEyeX = headEndConf[4] - headStartConf[4];
		double relEyeY = headEndConf[3] - headStartConf[3];
		double relHeadX = headEndConf[2] - headStartConf[2];
		double relHeadY = headEndConf[0] - headStartConf[0];

		relHeadX += eyeXtoHeadX(relEyeX);
		relHeadY += eyeYtoHeadY(relEyeY);
		if(headSac->isMotorField(relHeadX, relHeadY))
		{
			headInput = headSac->getMotorField(relHeadX, relHeadY);
			cout << "Making a head link between: " << *headInput << " and " << *headInput << endl;
			headSac->makeLink(headInput, headInput);
		}
		Time::delay(0.2);

	}
}




//bool EyeHeadController::simpleEyeOnlySaccade(string colour)
//{
//	eyeHeadlog << "Attempting simple saccade to target " << colour << endl;
//
//	double targX, targY;
//	if(!target->getTarget(&targX, &targY, colour))
//	{
//		cout << "Target not visible from here" << endl;
//		eyeHeadlog << "Target not visible from here" << endl;
//		return false;
//	}
//
//
//}
//bool EyeHeadController::simpleEyeHeadSaccade(string colour);


/**
 *  This will attempt to move the eyes towards the centre, without guaranteeing actual centring.
 */
bool EyeHeadSaccading::centreEyesInHead()
{
	double* headMotorConf = new double[6];
	bool success = headCont->getCurrentPosition(headMotorConf);

	if(success)
	{
		printf("Eye position started at (%.2f, %.2f)\n",headMotorConf[4],headMotorConf[3]);
		printf("Head position started at (%.2f, %.2f)\n",headMotorConf[2],headMotorConf[0]);
		headMotorConf[2] -= eyeXtoHeadX(headMotorConf[4]);	//pan
		headMotorConf[0] -= eyeYtoHeadY(headMotorConf[3]);	//tilt
//		headMotorConf[2] -= headMotorConf[4];	//pan
//		headMotorConf[0] += headMotorConf[3];	//tilt

		v->resetOverflow();
		boost::thread vorThrd(boost::bind(&vor::track,v));
		headCont->move(headMotorConf[2],headMotorConf[0],true);
		Time::delay(0.5);
		vorThrd.interrupt();

		headCont->getCurrentPosition(headMotorConf);
		printf("Eye position ended at (%.2f, %.2f)\n",headMotorConf[4],headMotorConf[3]);
		printf("Head position ended at (%.2f, %.2f)\n",headMotorConf[2],headMotorConf[0]);
	}
	return success;
}


/**
 * By default, this function will only make one attempt, however
 * repeated attempts can be made to fixate
 */
bool EyeHeadSaccading::centreEyesInHeadOnTarget(string colour, bool retry)
{
	double targX, targY;
//	string colour;
	target->getTarget(&targX, &targY, colour);
//	colour = target->getColour();
	bool success;

	//Get the eyes fixated on the target
	success = eyeSac->simpleSaccade(targX, targY, colour);
	if(!success && !retry)
	{
		cout << "Eyes failed to fixate on target" << endl;
	}
	int counter = 1;
	while(retry && !success)
	{
		target->getTarget(&targX, &targY, colour);
		success = eyeSac->simpleSaccade(targX, targY, colour);
		counter++;
	}
	if(retry)
		cout << "Eye fixated on target after " << counter << "attempts" << endl;


	double x,y;
	eyeCont->getCurrentPosition(&x, &y);

	v->resetOverflow();
	//activate vor thread
//	boost::thread vorThrd(*v);
	boost::thread vorThrd(boost::bind(&vor::track,v));



	success = headSac->simpleSaccade();
	vorThrd.interrupt();
	if(!retry)
		return success;

	counter = 1;
	while(retry && !success)
	{
//		v->resetOverflow();
//		boost::thread vorThrd2(*v);
		boost::thread vorThrd2(boost::bind(&vor::track,v));
		success = headSac->simpleSaccade();
		vorThrd2.interrupt();
		counter++;
	}
	cout << "Head centred eyes after " << counter << " attempts" << endl;

	success = target->targetCentred(&targX, &targY, colour);
	return success;
}


bool EyeHeadSaccading::verge(string colour, double* depth, bool centreHead)
{
	bool success = true;

	if(centreHead)
		centreEyesInHead();

	autoCenter(colour);

	eyeSac->verge(colour);
	eyeCont->getVergence(depth);

	return success;
}

bool EyeHeadSaccading::getVergence(double* vergence)
{
	return eyeCont->getVergence(vergence);
}


bool EyeHeadSaccading::checkInputFields(double targX, double targY, PolarField* eyeOutput, PolarField* headOutput)
{
	bool gotField;
	PolarField* eyeInput = new PolarField();
	gotField = eyeSac->isRetinaField(targX, targY);


	PolarField* headInput = new PolarField();
	gotField &= headSac->isRetinaField(targX, targY);


	if(gotField)
	{
		eyeInput = eyeSac->getRetinaField(targX, targY);
		headInput = headSac->getRetinaField(targX, targY);

		eyeOutput = eyeSac->getLinkedOutput(eyeInput);
		headOutput = headSac->getLinkedOutput(headInput);
	}
	return gotField;

}


bool EyeHeadSaccading::checkCombiLinks(double targX, double targY)
{
	string colour = target->getColour();
	return checkCombiLinks(targX, targY, colour);

}
bool EyeHeadSaccading::checkCombiLinks(double targX, double targY, string colour, bool check)
{
	double startTargX, startTargY;
	startTargX = targX;
	startTargY = targY;
	eyeHeadlog << "Target: " << targX << " " << targY << endl;


	if(check)
	{
		if(!target->targetVisible())
			return false;
	}

	PolarField* eyeOutput = new PolarField();
	PolarField* headOutput = new PolarField();

	double radius=1;
	bool gotField = eyeSac->isRetinaField(targX, targY);
	if(gotField)
	{
		PolarField *eyeLocal = eyeSac->getRetinaField(targX, targY);
		radius = eyeLocal->getRadius();
	}

	float eyeDist, headDist;
	PolarField* eyeInput = eyeSac->getNearestLearntInput(targX, targY, &eyeDist);
	eyeOutput = eyeSac->getLinkedOutput(eyeInput);
	if(!(eyeDist<(radius*1.5) && eyeDist>0 && eyeInput->getUsage()>=0))
		return false;

	eyeHeadlog << "Eye input field: " << *eyeInput << endl;
	eyeHeadlog << "Eye output field: " << *eyeOutput << endl;

	gotField = headSac->isRetinaField(targX, targY);
	if(gotField)
	{
		PolarField *headLocal = headSac->getRetinaField(targX, targY);
		radius = headLocal->getRadius();
	}
	PolarField* headInput = headSac->getNearestLearntInput(targX, targY, &headDist);
	headOutput = headSac->getLinkedOutput(headInput);
	if(!(headDist<(radius*1.5) && headDist>0 && headInput->getUsage()>=0 && eyeInput->getUsage()>=0))
		return false;

	eyeHeadlog << "Head input field: " << *headInput << endl;
	eyeHeadlog << "Head output field: " << *headOutput << endl;

//	if(!checkInputFields(targX, targY, eyeOutput, headOutput))
//		return false;


	cout << "***********************************************" << endl;
	cout << "* Found matching links, calculating movements *" << endl;
	cout << "***********************************************" << endl;
	eyeHeadlog << "***********************************************" << endl;
	eyeHeadlog << "* Found matching links, calculating movements *" << endl;
	eyeHeadlog << "***********************************************" << endl;
//		cout << "Target: " << targX << " " << targY << endl;
	eyeHeadlog << "Target: " << targX << " " << targY << endl;
	//calculate gaze displacement
	double gazeX = eyeOutput->getXcoord();
	double gazeY = eyeOutput->getYcoord();


//		cout << "eye: " << *eyeOutput << endl;
//		cout << "head: " << *headOutput << endl;
	eyeHeadlog << "eye: " << *eyeOutput << endl;
	eyeHeadlog << "head: " << *headOutput << endl;

	//initial eye offset
	double eyeXpos, eyeYpos;
	double eyeOffsetX, eyeOffsetY;
	bool gotPos = eyeCont->getCurrentPosition(&eyeXpos, &eyeYpos);
	//this should be relative to the gaze displacement
	// i.e. +ve if away, -ve if towards.

	double headXpos, headYpos;
	gotPos &= headCont->getCurrentPosition(&headXpos, &headYpos);

	if(!gotPos)
	{
		cout << "Failed to obtain current eye/head positions" << endl;
		eyeHeadlog << "Failed to obtain current eye/head positions" << endl;
		return false;
	}

//		cout << "Eye pos: (" << eyeXpos << ", " << eyeYpos << ")" << endl;
//		cout << "Head pos: (" << headXpos << ", " << headYpos << ")" << endl;
	eyeHeadlog << "Eye pos: (" << eyeXpos << ", " << eyeYpos << ")" << endl;
	eyeHeadlog << "Head pos: (" << headXpos << ", " << headYpos << ")" << endl;

	eyeOffsetX = eyeXpos;
	eyeOffsetY = eyeYpos;

	if(gazeX > 0 && eyeOffsetX > 0)	// eyes off centre in direction of gaze
		eyeOffsetX = -eyeOffsetX;
	else if(gazeX > 0 && eyeOffsetX < 0) // eyes off centre in opposite direction of gaze
		eyeOffsetX = -eyeOffsetX;

	if(gazeY > 0 && eyeOffsetY > 0)	// eyes off centre in direction of gaze
		eyeOffsetY = -eyeOffsetY;
	else if(gazeY > 0 && eyeOffsetY < 0) // eyes off centre in opposite direction of gaze
		eyeOffsetY = -eyeOffsetY;

//		cout << "Eye offset: (" << eyeOffsetX << ", " << eyeOffsetY << ")" << endl;
	eyeHeadlog << "Eye offset: (" << eyeOffsetX << ", " << eyeOffsetY << ")" << endl;

	//gaze should always be positive for the eye-head calculations
	gazeX = abs(gazeX);
	gazeY = abs(gazeY);

//		cout << "Gaze: (" << gazeX << ", " << gazeY << ")" << endl;
	eyeHeadlog << "Gaze: (" << gazeX << ", " << gazeY << ")" << endl;

	//calculate head contribution
	double headXpercent = headPercentH(gazeX, eyeOffsetX);
	double headYpercent = headPercentV(gazeY, eyeOffsetY);

	if(isinf(headXpercent) || isnan(headXpercent))
		headXpercent = 0;
	else if(headXpercent > 400)
		headXpercent = 400;

	if(isinf(headYpercent) || isnan(headYpercent))
		headYpercent = 0;
	else if(headYpercent > 400)
		headYpercent = 400;


	cout << "Head percent: " << headXpercent << "% , " << headYpercent << "% )"<< endl;
	eyeHeadlog << "Head percent: " << headXpercent << "% , " << headYpercent << "% )"<< endl;


	//follow links
	//if the eye movement is greater than the available range, need to add the
	//excess as overflow to the vor!
	double xDist, yDist;
	xDist = eyeOutput->getXcoord();	// relative motor values
	yDist = eyeOutput->getYcoord();
	double xOverflow, yOverflow;
	xOverflow = 0;
	yOverflow = 0;

	v->resetOverflow();

	if(calcOverflow(xDist, yDist, eyeXpos, eyeYpos,	&xOverflow, &yOverflow))
	{
		cout << "Adding overflow: (" << xOverflow << ", " << yOverflow << ")" << endl;
		eyeHeadlog << "Adding overflow: (" << xOverflow << ", " << yOverflow << ")" << endl;
		v->addOverflow(xOverflow, yOverflow);
	}

	double headXmovement = headOutput->getXcoord() * headXpercent / 100;
	double headYmovement = headOutput->getYcoord() * headYpercent / 100;

	if(headXpercent == 0 && headYpercent==0)
		eyeOnly++;

//		cout << "Head X movement: " << headXmovement << " Head Y movement: " << headYmovement << endl;
//		cout << "Eye X movement: " << eyeOutput->getXcoord() << " Eye Y movement: " << eyeOutput->getYcoord() << endl;
	eyeHeadlog << "Head X movement: " << headXmovement << " Head Y movement: " << headYmovement << endl;
	eyeHeadlog << "Eye X movement: " << eyeOutput->getXcoord() << " Eye Y movement: " << eyeOutput->getYcoord() << endl;

	eyeSac->followLink(eyeOutput->getXcoord(), eyeOutput->getYcoord());
//	boost::thread vorThrd(*v);
	boost::thread vorThrd(boost::bind(&vor::track,v));
	Time::delay(0.1);
	headSac->followLink(headXmovement, headYmovement);

	while(!(headSac->allStationary()))
	{
		Time::delay(0.4);
	}
	vorThrd.interrupt();
	Time::delay(0.4);
	if(check)
	{
		//evaluate
		if(!target->targetCentred(&targX,&targY, colour))
		{
			//Made a move, but the target wasn't centred
			cout << "eye-head movement failed to properly centre target" << endl;
			eyeHeadlog << "eye-head movement failed to properly centre target" << endl;

			double dist =0;
			target->fovea(targX, targY, &dist);
			if(dist<=32)
			{
				cout << "Target nearly centred" << endl;
				bool success = autoCenter(colour);
				combiClose ++;
				//make small eye saccade to target
	//			bool success = eyeSac->simpleSaccade(targX, targY, colour);
	//			bool success = eyeSac->saccade(saccadeCounter,targX,targY,colour);
				if(params.LEARN)
				{
					FieldLink* link = eye_ppm->getLink(eyeInput, eyeOutput);
					link->useField();

					eyeInput->useField();
					eyeOutput->useField();

					link = head_ppm->getLink(headInput, headOutput);
					link->useField();

					headInput->useField();
					headOutput->useField();

					if(success)
					{

						//Add field to gaze map
						double* headMotorPos = new double[6];
						bool gotEnc = headCont->getCurrentPosition(headMotorPos);
						if(gotEnc)
						{
							cout << "Attempting to add a gaze field" << endl;
							gm->addGazeField(headMotorPos);
						}
						else
							cout << "didn't get the full array of motor positions" << endl;
					}
				}
				return success;
			}
			else
			{
				FieldLink* link = eye_ppm->getLink(eyeInput, eyeOutput);
				link->linkFailed();
				eyeInput->linkFailed();
				eyeOutput->linkFailed();
				if(!(headXpercent == 0 && headYpercent==0))
				{
					link = head_ppm->getLink(headInput, headOutput);
					link->linkFailed();
					headInput->linkFailed();
					headOutput->linkFailed();
				}
				combiFailed ++;
				//did have some checking and link updating here,
				//but the links added seemed to be more erratic
				//so this has been removed.  May want to do something
				//here in the future though.
				return false;
			}


		}
		else
		{
			autoCenter(colour);
			//Increment usage counts for all of the fields involved
			if(params.LEARN)
			{
				PolarField* eyeInput = new PolarField();
				PolarField* headInput = new PolarField();
				eyeInput = eyeSac->getLinkedInput(eyeOutput);
				headInput = headSac->getLinkedInput(headOutput);

				eyeOutput = eyeSac->getLinkedOutput(eyeInput);
				headOutput = headSac->getLinkedOutput(headInput);

				FieldLink* link = eye_ppm->getLink(eyeInput, eyeOutput);
				link->useField();
				link = head_ppm->getLink(headInput, headOutput);
				link->useField();


				eyeInput->useField();
				headInput->useField();
				eyeOutput->useField();
				headOutput->useField();

				//Add field to gaze map
				double* headMotorPos = new double[6];
				bool gotEnc = headCont->getCurrentPosition(headMotorPos);
				if(gotEnc)
				{
					cout << "Attempting to add a gaze field" << endl;
					gm->addGazeField(headMotorPos);
				}
				else
					cout << "didn't get the full array of motor positions" << endl;
			}
			cout << "eye+head movement succeeded!" << endl;
			eyeHeadlog << "eye+head movement succeeded!" << endl;
			combiSuccessful ++;
			return true;
		}
	}
	else	//can't check, but a move was made.
	{
		return true;
	}

	return false;
}


bool EyeHeadSaccading::addGazeField()
{
	//Add field to gaze map
	cout << "Attempting to add a gaze field" << endl;
	double* headMotorPos = new double[6];

	bool gotEnc = headCont->getCurrentPosition(headMotorPos);
	if(gotEnc)
	{
		cout << "Adding gaze field" << endl;
		gm->addGazeField(headMotorPos);
	}
	else
		cout << "Didn't get the full array of head motor positions" << endl;

	return gotEnc;
}

void EyeHeadSaccading::goToHeadConfig(HeadConfig* hc)
{
	double* config = new double[6];
	hc->toArray(config);

	eyeCont->move(config[eyeX], config[eyeY], false);
//	if((int)config[5] != 0)
		eyeCont->verg(config[5],false);
	headCont->move(config[headX], config[headY], true);

}


bool EyeHeadSaccading::goToGazeField(GazeField* gf)
{
	HeadConfig* hc = gf->getPreferredGaze();

	goToHeadConfig(hc);
	return true;
}

bool EyeHeadSaccading::goToGazeField(double gazeX, double gazeY)
{
	GazeField* gf = gm->getGazeField(gazeX, gazeY);
	if(gf->getXcoord()==0 && gf->getYcoord()==0)		//probably null field returned
	{
//		eyeCont->toRest();
//		headCont->toRest();
		return false;
	}
	return goToGazeField(gf);
}

bool EyeHeadSaccading::isGazeField()
{
	double* headMotorConfig = new double[6];
	headCont->getCurrentPosition(headMotorConfig);
	GazeField* g = gm->getGazeField(headMotorConfig);
	if(g->getXcoord()!=0 && g->getYcoord()!=0)
		return true;
	else
		return false;
}

GazeField* EyeHeadSaccading::getGazeField()
{
	double* headMotorConfig = new double[6];
	bool success = headCont->getCurrentPosition(headMotorConfig);
	if(!success)
	{
		cout << "Failed to obtain head motor configuration" << endl;
		return new GazeField();
	}
	printf("HMC: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]\n",headMotorConfig[0],headMotorConfig[1],headMotorConfig[2],headMotorConfig[3],headMotorConfig[4],headMotorConfig[5]);
	GazeField* gf = gm->getGazeField(headMotorConfig);

	if(gf->getXcoord() == 0 && gf->getYcoord()==0)
	{
		cout << "Gaze field not found, adding new field" << endl;
		gm->addGazeField(headMotorConfig);
		gf = gm->getGazeField(headMotorConfig);
	}
	return gf;
}


void EyeHeadSaccading::getGazeDirection(double* gazeX, double* gazeY)
{
	double* headMotorConfig = new double[6];
	headCont->getCurrentPosition(headMotorConfig);
	gm->gazeData(headMotorConfig, gazeX, gazeY);
}



bool EyeHeadSaccading::getGazeDirection(double targX, double targY, double* headMotorConfig)
{
	bool check = headCont->getCurrentPosition(headMotorConfig);
	if(check)
	{
		float dist;
		PolarField* input = eyeSac->getNearestReliableLearntInput(targX, targY, &dist);
		printf("input field reliability: %i\n",input->getUsage());
		if(dist<16)
		{
			PolarField* output = eyeSac->getLinkedOutput(input);
			double relX, relY, x,y;
			relX = output->getXcoord();
			relY = output->getYcoord();

			headMotorConfig[eyeX] += relX;
			headMotorConfig[eyeY] += relY;
		}
		else
			check=false;
	}
	return check;
}

bool EyeHeadSaccading::getGazeDirection(double targX, double targY, double* gazeX, double* gazeY)
{
	double* headMotorConfig = new double[6];
	bool check = headCont->getCurrentPosition(headMotorConfig);
	if(check)
	{
		float dist;
		PolarField* input = eyeSac->getNearestReliableLearntInput(targX, targY, &dist);
		printf("input field reliability: %i\n",input->getUsage());
		if(dist<16)
		{
			PolarField* output = eyeSac->getLinkedOutput(input);
			double relX, relY, x,y;
			relX = output->getXcoord();
			relY = output->getYcoord();

			headMotorConfig[eyeX] += relX;
			headMotorConfig[eyeY] += relY;
			gm->gazeData(headMotorConfig, gazeX, gazeY);
		}
		else
			check=false;
	}
	return check;
}

bool EyeHeadSaccading::getDepth(string colour, double* depth)
{
	return eyeSac->getDepth(colour, depth);
}



bool EyeHeadSaccading::autoCenter(std::string colour)
{
	double targX, targY;
	target->getTarget(&targX,&targY,colour);
	if(!target->targetVisible())
		return false;
	return eyeSac->autoCenter(targX,targY, colour);
}


void EyeHeadSaccading::headStats()
{
	int unlearntInputfields;
	int unlearntOutputfields;

	int numLinks, numGoodLinks;

	time_t timeStamp;
	timeStamp = time(NULL);

	unlearntInputfields = head_ppm->unlearntInputFields();
	unlearntOutputfields = head_ppm->unlearntOutputFields();

	numLinks = head_ppm->getNumLinks();
	numGoodLinks = head_ppm->getNumGoodLinks();

	headstatslog << saccadeCounter << " " << eyeOnly << " "<< combiSuccessful << " "<< combiClose << " "<< combiFailed
				<< " " << unlearntInputfields << " " << unlearntOutputfields << " " << eyeHeadSuccessRate << " "
				<< numLinks << " " << numGoodLinks << " " << timeStamp << endl;

}


void EyeHeadSaccading::eyeStats(bool links)
{
	//Eye stats
	int saccadecounter;
	int stepcounter;
	bool successfulDirectLink;
	bool successfulNeighbourLink;
	int unsuccesfulDirectLinkCounter;
	int neighbourCounter;
	bool penultimateMoveWasLink;
	int linksLearnt;
	int linksUpdated;
	int possibleLinkstoLearn;
	int unlearntInputfields;
	int unlearntOutputfields;


	int numLinks, numGoodLinks;


	time_t timeStamp;
	timeStamp = time(NULL);

	if(links)
	{
		saccadecounter = saccadeCounter;
		stepcounter = 1;
		successfulDirectLink = true;
		successfulNeighbourLink = false;
		unsuccesfulDirectLinkCounter = 0;
		neighbourCounter = 0;
		penultimateMoveWasLink = false;
		linksLearnt = 0;
		linksUpdated = 0;
		possibleLinkstoLearn = 0;
	}
	else
	{
		eyeSac->getStats(&saccadecounter, &stepcounter, &successfulDirectLink, &successfulNeighbourLink,
			&unsuccesfulDirectLinkCounter, &neighbourCounter, &penultimateMoveWasLink,
			&linksLearnt, &linksUpdated, &possibleLinkstoLearn);
	}

	unlearntInputfields = eye_ppm->unlearntInputFields();
	unlearntOutputfields = eye_ppm->unlearntOutputFields();

	numLinks = eye_ppm->getNumLinks();
	numGoodLinks = eye_ppm->getNumGoodLinks();

//	unlearntfields = eye_ppm->unlearntInputFields();

	if(rollingBlock.size() < 10)
	{
		rollingBlock.push_back(stepcounter);
	}
	else
	{
		rollingBlock.pop_front();
		rollingBlock.push_back(stepcounter);
	}

	rollingAverage = 0;
	list<int>::iterator it;

	for ( it=rollingBlock.begin() ; it != rollingBlock.end(); it++ )
	{
		rollingAverage += *it;
	}
	rollingAverage /= rollingBlock.size();

	cout << "Rolling average: " <<  rollingAverage << endl;


	eyestatslog << saccadecounter << " " << stepcounter << " " << successfulDirectLink << " "
			<< successfulNeighbourLink << " " << unsuccesfulDirectLinkCounter << " "
			<< neighbourCounter << " " << penultimateMoveWasLink << " " << linksLearnt << " "
			<< linksUpdated << " " << possibleLinkstoLearn << " " << unlearntInputfields << " "
			<< unlearntOutputfields << " " << rollingAverage << " " << numLinks << " " << numGoodLinks <<
			" " << timeStamp << endl;

	if(saccadecounter<5 && rollingAverage<eyeThreshold)
		rollingAverage += 5;	//artificially inflate rolling average until done at least a few saccades, to ensure reasonable mapping


}

void EyeHeadSaccading::closeLogs()
{
	target->closeLog();
	eyeSac->closeLogs();
	headSac->closeLogs();


	eyestatslog << "**************END**************" << endl;
	eyestatslog.close();

	headstatslog << "**************END**************" << endl;
	headstatslog.close();

	eyeHeadlog << "**************END**************" << endl;
	eyeHeadlog.close();

	headLinkLog.close();
}




/****************************************************************************************************
 * Test functions
 **************************************************************************************************** */

void EyeHeadSaccading::testMotorLinks()
{
	PolarField* m1 = eyeSac->getMotorField(30, 30);
	cout << "obtained motor field (m1): " << *m1 << endl;

	m1->setUsage(51);
	cout << "updated field usage: " << *m1 << endl;

	PolarField* r1 = eyeSac->getRetinaField(20, 20);
	cout << "obtained retina field (r1): " << *r1 << endl;

	r1->setUsage(51);
	cout << "updated field usage: " << *r1 << endl;


	eyeSac->makeLink(r1,m1,42);

	float dist;
	PolarField* r2 = eyeSac->getNearestLearntInput(15, 10, &dist);
	cout << "Obtained a retina field at distance " << dist << " that should be r1: " << *r2 << endl;

	PolarField* m2 = eyeSac->getLinkedOutput(r2);
	cout << "Obtained linked motor field that should be m1: " << *m2 << endl;

	m2->setUsage(42);
	cout << "Updated usage of m2: " << *m2 << endl;

	cout << "Curious about m1: " << *m1 << endl;

	PolarField* m3 = eyeSac->getNearestLearntOutput(45, 15, &dist);
	cout << "Obtained a learnt motor field at distance " << dist << " that should be m1: " << *m3 << endl;

	m3->setUsage(666);
	cout << "Updated usage of m3: " << *m3 << endl;
	cout << "Curious about m1: " << *m1 << endl;
	cout << "Curious about m2: " << *m2 << endl;

	PolarField* m4 = eyeSac->getMotorField(30, 30);
	cout << "Final test: " << *m4 << endl;


	PolarField* m5 = (PolarField*)eye_ppm->getOutputField(30,30);
	cout << "From eye_ppm: " << *m5 << endl;

}


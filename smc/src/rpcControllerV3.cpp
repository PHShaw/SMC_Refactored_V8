/*
 * rpcController.cpp
 *
 * Provides an RPC interface to the SMc functionality
 *
 * NOTE: Gaze and reach coordinates are now recorded in x,y,z world coords in schemas now.
 * TODO: include VAM to expand on object identification
 *
 *  Created on: 10/03/2012
 *  Author: Michael Sheldon <mts08@aber.ac.uk>
 */

#include <yarp/os/all.h>

#include <signal.h>

#include "EyeHeadController.h"
#include "handEyeCoordination.h"
#include "tracker.h"
#include "graspController.h"
#include "torsoSaccading.h"

#include "ShortTermMemoryV2.h"

#define OCCLUSION_THRESHOLD 0.5
#define GRASP_DISTANCE_THRESHOLD 0.1
#define MAX_OBJECTS 5	//Only used in simulated version
#define HAND_ID 2
#define NEAREST_REACH_RADIUS 2
#define REACH_THRESHOLD 0.03

//armReaching* armReach;
armController* ac;
EyeHeadSaccading* ehCont;
//handEyeCoordination* heCoor;
graspController *grippy;
torsoSaccading* tor;
//TorsoReach* torRoar;
Target* target;
GazeMap* gm;
yarp::os::Bottle* prevTargetData;
ShortTermMemory* stm;
const char *holdingType = NULL;
int holdingObject = -1;
int holdingObjectId = -1;
int objects[6] = {-1, -1, -1, -1, -1, -1};
int depths[6] = {-1, -1, -1, -1, -1, -1};
int prevSize[6] = {-1, -1, -1, -1, -1, -1};
int prevDepth[6] = {-1, -1, -1, -1, -1, -1};
std::map<int, GazeField*> prevFields;

const int reachDepthThreshold = 70;
const bool rightArm=false;

var_params params;
//string path;
//string filename;

/*
 * In experiments for sequential vs synchronous learning, these variable should be changed!
 */
//bool synchronous = false;
//bool nearestNeighbour = true;
void reach_to_target(yarp::os::Bottle *reply, double x, double y, double z, int arm, bool point=false);
yarp::os::BufferedPort<yarp::os::Bottle> tport;

yarp::os::BufferedPort<yarp::os::Bottle> portOut;
yarp::os::BufferedPort<yarp::os::Bottle> portIn;
yarp::os::Bottle* reachResponse;

//1. Run $ICUB_ROOT/app/faceExpressions/scripts/app.xml.template
//2. Run ./emotionController
enum emote {neu,hap1,hap2,sad,sur,ang,evi,shy,cun};
yarp::os::BufferedPort<yarp::os::Bottle> emotePort;
void write(emote e)
{
	yarp::os::Bottle& out = emotePort.prepare();
	out.clear();
	out.addInt(e);
	cout << "Sending int " << e << endl;
	emotePort.write(true);
	yarp::os::Time::delay(0.001);
}

void loadGazeReachMap()
{
	Gaze_IO gm_io;
	delete gm;
	gm = gm_io.loadMappingFromXML(params.m_PATH + "GM_" + params.m_FILENAME);
	cout << "There are " << gm->getNumGazeFields() << " fields in the gaze map" << endl;

}
void saveGazeMap()
{
	Gaze_IO gmIO;
	try{
		cout << "There are " << gm->getNumGazeFields() << " gaze fields to save" << endl;
		gmIO.saveMappingToXML(gm, params.m_PATH+"GM_" + params.m_FILENAME);
		cout << "Successfully saved: ";
		cout << gm->getNumGazeFields() << " Gaze fields, ";
//		cout << gm->getNumReachFields() << " Reach fields and ";
//		cout << gm->getNumLinks() << " links " << endl;
	}
	catch(IMapException ime)
	{
		cout << "An error occurred whilst attempting to save the gaze map"<<endl;
	}
}


void saveAndQuit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
//	heCoor->closeLogs();

	ehCont->saveMaps();
	tor->saveMapping();
//	heCoor->saveGazeReachMap();
	saveGazeMap();
	tor->closeMatlabPorts();

	std::exit(param);
}


void quit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
	tor->closeMatlabPorts();
	delete ehCont;
	delete tor;
	std::exit(param);
}


int colourToId(const char *colour) {
	if (strcmp(colour, "red") == 0)		return 1;
	if (strcmp(colour, "yellow") == 0)	return 2;
	if (strcmp(colour, "green") == 0)	return 3;
	if (strcmp(colour, "blue") == 0)	return 4;
	if (strcmp(colour, "white") == 0)	return 5;
}

string idToColour(const int id) {
	if (id==1) return "red";
	if (id==2) return "yellow";
	if (id==3) return "green";
	if (id==4) return "blue";
	if (id==5) return "white";
}

//TODO

void add_visual_targets(yarp::os::Bottle *reply) {
	stm->update(holdingObjectId);
	stm->printMemory();
	stm->reportMemory(reply, holdingObjectId);
}

void add_visual_targets_old(yarp::os::Bottle *reply) {
	yarp::os::Bottle *targetData = NULL;

	targetData = tport.read(0);
	if (targetData == NULL) {
		return;
	}

	for(int i=0; i<6; i++)
	{
		objects[i] = -1;
	}

	//check for torso offset
	double* torsoPose = new double[3];
	tor->getTorsoController()->getCurrentPosition(torsoPose);
	double tilt, version;
	bool torComp=false;
	if((int)(torsoPose[0])!=0)
	{
		printf("Thinking about torso contribution, based on %.2f\n", torsoPose[0]);
//		//some torso rotation has been applied.
//		double* torsoZero = new double[3];
//		double* headZero = new double[6];
//		double vergence;
//		CalculateGazeChange(torsoZero, headZero,torsoPose, headZero, &tilt, &version, &vergence);
//		torComp=true;
	}

	// Fixate on each possible target in turn
	for(int t = 0; t < targetData->size(); t += 4) {
		GazeField *field = NULL;
		// Get target information
		yarp::os::ConstString colour = targetData->get(t).asString();
		int id = colourToId(colour.c_str());
		double targX = targetData->get(t+1).asDouble();
		double targY = targetData->get(t+2).asDouble();
		int size = targetData->get(t+3).asInt();
		double saliency = 1.0; // TODO: retrieve this from short-term memory
		double depth;

		if(size<=5)
		{
			printf("Size of %i judged to be insignificant so dismissing this colour\n", size);
			continue;
		}
		else
		{
			printf("Object of size %i found\n",size);
		}
		bool success = false;
		if (id == holdingObjectId) {
			field = prevFields[HAND_ID];
			size = prevSize[id];
			success = true;
			depth = prevDepth[HAND_ID];
		}

		if(prevTargetData != NULL && field == NULL) {
			for (int t2 = 0; t2 < prevTargetData->size(); t2 += 4) {
				// See if we're occluding an object we saw previously
				printf("Checking previous colour: %s\n", prevTargetData->get(t2).asString().c_str());
				if (colour == prevTargetData->get(t2).asString()) {
					printf("Previous size: %d, Current size: %d\n", prevSize[id], size);
					// Assume our hand can't be occluded
					if (size < (prevSize[id] * OCCLUSION_THRESHOLD) && id != HAND_ID) {
						field = prevFields[id];
						size = prevSize[id];
						success = true;
						depth=prevDepth[id];
					} else if(prevSize[id]>0) {
						// Check to see if the object is still in that position
						GazeField *testField = prevFields[id];
						if(torComp)
						{
							double x,y, dist;
							testField->getGazeDirection(&x, &y);
							x+=version;
							y+=tilt;
							testField = gm->getGazeField(x,y);
							if(testField->getXcoord() == 0 && testField->getYcoord()==0)
							{
								printf("test field is null, trying to get nearest\n");
								gm->getNearestGazeField(x,y,&testField,&dist);
								cout << "Obtained: " << *testField << endl;
							}
						}
						ehCont->goToGazeField(testField);
						sleep(1);
						success = target->targetCentred(&targX, &targY, colour.c_str());
						if(success) {
							ehCont->centreEyesInHead();
							ehCont->autoCenter(colour.c_str());
							field = ehCont->getGazeField();
							if(torComp)
							{
								double x,y, dist;
								field->getGazeDirection(&x, &y);
								x-=version;
								y-=tilt;
								field = gm->getGazeField(x,y);
								if(field->getXcoord() == 0 && field->getYcoord()==0)
									gm->getNearestGazeField(x,y,&field,&dist);
							}
//							field = testField;
							ehCont->getDepth(colour.c_str(), &depth);
							printf("object at depth: %.2f\n",depth);
						}
					}
				}
			}
		}
		vector<GazeReachLink*> links = gm->getLinks();
		int babble_field = 0;
		while(!success && babble_field < links.size() ) {
			if(target->targetVisible()) {
				ehCont->fixate(targX, targY, colour.c_str(), true);
			}
			sleep(1);
			success = target->targetCentred(&targX, &targY, colour.c_str());
			if(!success) {
				GazeReachLink *l = links.at(babble_field);
				ReachField *rf = l->reach; //gm->getReachField(l->depth, l->angle);
				GazeField *gf = l->gaze;	//gm->getGazeField(l->gazeX, l->gazeY);
				ehCont->goToGazeField(gf);
				babble_field++;
			}
			else
			{
				ehCont->centreEyesInHead();
				ehCont->autoCenter(colour.c_str());
				ehCont->getDepth(colour.c_str(), &depth);
				printf("object at depth: %.2f\n",depth);
			}
		}
		if(!success) {
			// Object is in a field we can't interact with, so ignore it
			continue;
		}
		if(field == NULL) {
			if(ehCont->isGazeField()) {
				// Find field in gaze space
				field = ehCont->getGazeField();
			} else if(ehCont->addGazeField()) {
				cout << "Created a field" << endl;
				field = ehCont->getGazeField();
			} else {
				cout << "Failed to find or create a field" << endl;
				// Couldn't find/create a gaze field
				continue;
			}

			//TODO
//			if(torComp)
//			{
//				double x,y, dist;
//				field->getGazeDirection(&x, &y);
//				x-=version;
//				y-=tilt;
//				field = gm->getGazeField(x,y);
//				if(field->getXcoord() == 0 && field->getYcoord()==0)
//					gm->getNearestGazeField(x,y,&field,&dist);
//			}
		}
		reply->addString("visual");
		reply->addInt(id);
		reply->addString(colour.c_str());
		double gazeX, gazeY;
		field->getGazeDirection(&gazeX, &gazeY);
		reply->addDouble(gazeX);
		reply->addDouble(gazeY);
		reply->addInt(size);
		reply->addDouble(saliency);

		if(depth<reachDepthThreshold)
				reply->addInt(1);		//predicted to be out of reach
			else							//vice versa
				reply->addInt(0);		//within reach

		objects[id] = field->getIndex();
		depths[id] = depth;
		prevFields[id] = field;
		prevSize[id] = size;
		prevDepth[id] = depth;
	}

	for(int i=1;i<6;i++)
	{
		double saliency = 1.0; // TODO: retrieve this from short-term memory
		if(i!=2 && objects[2]!=-1 && prevFields[i]!=NULL && objects[i]==-1 && prevFields[i] == prevFields[2])
		{
			printf("Reporting on an object obscured by the hand\n");
			objects[i] = objects[2];
			int size = prevSize[i];
			depths[i] = prevDepth[i];
			string colour = idToColour(i);
			int id = i;
			GazeField *field = prevFields[2];

			reply->addString("visual");
			reply->addInt(id);
			reply->addString(colour.c_str());
			double gazeX, gazeY;
			field->getGazeDirection(&gazeX, &gazeY);
			reply->addDouble(gazeX);
			reply->addDouble(gazeY);
			reply->addInt(size);
			reply->addDouble(saliency);

			if(depths[i]<reachDepthThreshold)
				reply->addInt(1);		//predicted to be out of reach
			else							//vice versa
				reply->addInt(0);		//within reach

		}
	}
	prevTargetData = targetData;

	printf("depths: [");
	for(int i=0; i<6; i++)
		printf("%i,",depths[i]);
	printf("]\n");

	printf("prevDepth: [");
	for(int i=0; i<6; i++)
		printf("%i,",prevDepth[i]);
	printf("]\n");
}


void add_touches(yarp::os::Bottle *reply) {
	double saliency = 1.0; // TODO: retrieve from short-term memory
	if (holdingObjectId == HAND_ID) {
		// We tried to grasp but only touched our own hand
		reply->addString("touch");
		reply->addInt(holdingObjectId);
		reply->addDouble(saliency);
		holdingObjectId = -1;
	} else if (holdingObjectId != -1) {
		reply->addString("holding");
		reply->addInt(holdingObjectId);
		reply->addDouble(saliency);
	} else {
		stm->getVisualTouch(reply);
	}
}

void add_touches_old(yarp::os::Bottle *reply) {
	double saliency = 1.0; // TODO: retrieve from short-term memory
	if (holdingObjectId == HAND_ID) {
		// We tried to grasp but only touched our own hand
		reply->addString("touch");
		reply->addInt(holdingObjectId);
		reply->addDouble(saliency);
		holdingObjectId = -1;
	} else if (holdingObjectId != -1) {
		reply->addString("holding");
		reply->addInt(holdingObjectId);
		reply->addDouble(saliency);
	} else {
		for(int i = 1; i < 6; i++) {
			if (i != 2 && objects[i] != -1 && objects[i] == objects[2]) {
				if(depths[i]<reachDepthThreshold)
				//if(abs(depths[2]-depths[i]) < 5)
				{
					reply->addString("touch");
					reply->addInt(i);
					reply->addDouble(saliency);
					printf("I think I am touching, hand: %i, object: %i\n",depths[2],depths[i]);
				}
				else
				{
					reply->addString("point");
					reply->addInt(i);
					reply->addDouble(saliency);
					printf("I think I am only pointing, hand: %i, object: %i\n",depths[2],depths[i]);
				}
			}
		}
	}
}


//bool nearest_reach_for_gaze(GazeField *gf, ReachField *rf) {
//	double correlation;
//	double *headMotorConfig = new double[6];
//	HeadConfig *hc = gf->getPreferredGaze();
//	hc->toArray(headMotorConfig);
//	double dist;
//	bool found = heCoor->getNearestReachForGaze(gf, &rf, &dist);	//gm->getNearestReachFieldForGaze(hc, rf, &correlation, NEAREST_REACH_RADIUS);
//	return found;
//}

/*
 * to work with the new reaching, going to supply a set of reaches (in world coords) within the available reach space at 5 cm intervals.
 * Need to specify left/right arm, so need to include this in the schema representation
 * Reaches can be done to a horizontal or vertical orientation, but not included this just yet
 */
void get_known_reaches(yarp::os::Bottle *reply) {
	//Reach space range (mm):
	// X: -380 -> -200
	// Y(L): -400 -> 0	Y(R): 0 -> 400
	// Z: 0 -> 400

	//at 5cm intervals this give 384 reach positions!
	//at 8cm intervals in y and z, this reduces to 150
	//Do I need to add in gaze only fields for pointing (torso use)?

	for(double x=-340; x<=-260; x+=40)
	{
		for(double z=0; z<=280; z+=75)
		{
			//Left
			for(double y=-300; y<=0; y+=75)
			{
				cout << "Adding reach " << x << ", " << y << ", " << z << ", left" << endl;
				reply->addDouble(x);
				reply->addDouble(y);
				reply->addDouble(z);
				reply->addInt(0);	//left arm
			}
			//right
			for(double y=0; y<=300; y+=80)
			{
				cout << "Adding reach " << x << ", " << y << ", " << z << ", right" << endl;
				reply->addDouble(x);
				reply->addDouble(y);
				reply->addDouble(z);
				reply->addInt(1);	//right arm
			}
		}
	}

}

bool last_arm = false;

void reach_to_target(yarp::os::Bottle *reply, double x, double y, double z, int arm, bool point) {

	if(point)
	{	//use torso to bring target into reach space

	}
	if(last_arm != arm)
	{
		yarp::os::Bottle& b = portOut.prepare();
		b.clear();
		b.addString("home");
		if(last_arm)
			b.addString("right_arm");
		else
			b.addString("left_arm");
		portOut.write();
		yarp::os::Time::delay(2.0);
	}
	yarp::os::Bottle& b = portOut.prepare();
	b.clear();
	b.addString("target");
	if(arm)
		b.addString("right_arm");
	else
		b.addString("left_arm");
	b.addDouble(x/1000);	//TODO: May want to add in an offset here, for safety... or check actual target coords at that location
	b.addDouble(y/1000);
	b.addDouble(z/1000);
	b.addString("horizontal");
	portOut.write();

	last_arm = arm;

	GazeField *gf = gm->getGazeField(x, y, z);
	yarp::os::Time::delay(0.01);
	write(neu);
	yarp::os::Time::delay(0.01);
	bool locateHand=false;
	if(gf->getXcoord()==0 && gf->getYcoord()==0)
	{
		double dist;
		bool success = gm->getNearestGazeField(x,y,z,&gf,&dist,100);
		cout << "Gaze field located at distance: " << dist <<endl;
		if(!success)
		{
			//Need to look for hand then add gaze field
			locateHand=true;
		}
	}

	if(!locateHand)
	{
		ehCont->goToGazeField(gf);
	}
	//cout << "obtained gaze field: " << *gf << endl;

	std::string message;
	bool armSuccess=false;
	double dist;
	do{
		yarp::os::Time::delay(0.3);
		reachResponse=portIn.read(true);
		message = reachResponse->get(0).asString().c_str();
	}while(message.compare("complete")!=0  && message.compare("unreachable")!=0);
	if(message.compare("complete")==0)
	{
		dist = reachResponse->get(1).asDouble();
		cout << "Reach distance: " << dist;
		if(dist<REACH_THRESHOLD)
			armSuccess=true;
	}
	else
		dist =1000;

	bool eye_success = true;
	if(locateHand && armSuccess)
	{
		eye_success = ehCont->fixate("yellow");	//will add gazeField if successful
		if(!eye_success)
		{
			if(last_arm)//right arm
			{
				ehCont->getHeadController()->move(-20,-20,true);
				ehCont->getEyeController()->move(10,-15, true);
			}
			else
			{
				ehCont->getHeadController()->move(20,-20,true);
				ehCont->getEyeController()->move(-10,-15, true);
			}
			eye_success = ehCont->fixate("yellow");
		}

		if(eye_success)
		{
			double depth;
			ehCont->verge("yellow", &depth);
			gf = ehCont->getGazeField();
		}
	}

	if(eye_success && armSuccess)
	{
		// Update our memory of where we last saw the hand
		//prevFields[HAND_ID] = gf;
		stm->updateHandPosition(gf);

		reply->addString("SUCCESS");
		yarp::os::Time::delay(0.01);
		write(hap1);
	}
	else
	{
		reply->addString("FAILED");
		yarp::os::Time::delay(0.01);
		write(sad);
	}
}


void press(yarp::os::Bottle *reply) {
	ac->pressButton(rightArm);
	reply->addString("SUCCESS");
	yarp::os::Time::delay(0.01);
	write(hap2);
}


void saccade(yarp::os::Bottle *reply, double x, double y, double z) {// int destinationField) {
	GazeField *gf = gm->getGazeField(x, y, z);	//gm->getGazeFields().at(destinationField);
	bool success;
	if(gf->getXcoord()==0 && gf->getYcoord()==0)
	{
		double dist;
		success = gm->getNearestGazeField(x,y,z,&gf,&dist,100);
		cout << "Found gaze field at distance: " << dist << endl;
	}
	else
	{
		success = true;
	}


	if(success)
	{
		ehCont->goToGazeField(gf);
		reply->addString("SUCCESS");
		yarp::os::Time::delay(0.01);
		write(hap1);
	}
	else
	{
		reply->addString("FAILED");
		write(sad);
	}
}


void grasp(yarp::os::Bottle *reply, bool release) {

	GazeField* field = stm->getHandGaze();		//prevFields[HAND_ID];
	if(field!=NULL)
	{
		if(!(field->getXcoord()==0 && field->getYcoord()==0))
		{
			ehCont->goToGazeField(field);
		}
	}

	yarp::os::Time::delay(0.01);
	write(neu);

	if(!release)
	{
		if(!grippy->isHolding())
		{
			bool success = grippy->grasp(rightArm);
			if(success) {
				vector<string> cols = target->getNearestObjects();
				bool found = false;
				for(int i=0; i<cols.size(); i++)
				{
					if(strcmp(cols.at(i).c_str(), YELLOW.c_str()) != 0)
					{
						holdingObjectId = colourToId(cols.at(i).c_str());
						found = true;
							break;
					}
				}
				if(!found)
				{
					holdingObjectId = stm->getNewHoldingID();
//					for (int i = 1; i < 6; i++) {
//						if (i != 2 && objects[i] == objects[2]) {
//							holdingObjectId = i;
//						}
//					}
				}

				if(holdingObjectId==-1)
				{
					printf("Failed to identify which object we were picking up\n");
					holdingObjectId = HAND_ID;
					grippy->release(rightArm);
					reply->addString("FAILED");
					yarp::os::Time::delay(0.01);
					write(sad);
					return;
				}
			} else {
				// We failed to grasp and are touching our own hand
				holdingObjectId = HAND_ID;
				grippy->release(rightArm);
				reply->addString("FAILED");
				yarp::os::Time::delay(0.01);
				write(sad);
				return;
			}
		}

	} else {
		grippy->release(rightArm);
		yarp::os::Time::delay(1.5);
		holdingObjectId = -1;
	}
//	armReach->toRest();

	reply->addString("SUCCESS");
	yarp::os::Time::delay(0.01);
	write(hap2);
}


void get_speech_excitation(yarp::os::Bottle *reply, const char *word) {
	reply->addDouble(1.0);
}


int main(int argc, char* argv[])
{
	params.LEARN = false;

	srand(NULL);

	yarp::os::Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	yarp::os::Value* val;
//	std::string robot;
	if(options.check("robot",val))
	{
		params.m_ROBOT = val->asString().c_str();
		cout << "Selected robot: " << params.m_ROBOT << endl;
	}
	else
	{
		cout << "A robot can be specified from the command line e.g. --robot [icub|icubSim]+F" << endl;
		params.m_ROBOT = "icub";
	}
	bool load = true;

	if(options.check("path",val))
	{
		params.m_PATH = val->asString().c_str();
		cout << "Loading files from path: " << params.m_PATH << endl;

		if(options.check("name",val))
		{
			params.m_FILENAME = val->asString().c_str();
			cout << "Loading file set: " << params.m_FILENAME << endl;
		}
		else
		{
			cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> params.m_FILENAME;
		}
	}
	else
	{

		cout << "Enter the path to the directory containing the files: e.g. ../../data/ " <<endl;
		cin >> params.m_PATH;
		cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
		cin >> params.m_FILENAME;
	}

	yarp::os::Network yarp;
	tport.open("/smc/target_read");
	target = new Target();

	//TODO: VAM
	target->initLog(params.m_PATH);
	loadGazeReachMap();

	ehCont = new EyeHeadSaccading(gm, target);
	tor = new torsoSaccading(ehCont, target);
//	armReach = new armReaching(robot, gm, true, true);
	ac = new armController(true);
	grippy = new graspController(params.m_ROBOT, ac);

	printf("Initiating short term memory...\n");
	stm = new ShortTermMemory(target, ehCont);//, tor);
	signal(SIGINT, saveAndQuit);
//	heCoor->init(ehCont, armReach, target, true);

	printf("Opening rpc port...\n");
//	RpcServer port;
	yarp::os::RpcServer port;
	port.open("/smc/rpc");
	yarp.connect("/schema/smc", "/smc/rpc");
	cout << "Server ready." << endl;


	printf("Opening emote port...\n");
	emotePort.open("/emote/ask");
	yarp.connect("/emote/ask", "/emote/emotion");



	portOut.open("/smc/reaching/out");
	portIn.open("/smc/reaching/in");
	yarp.connect("/smc/reaching/out", "/aber/reach/control:i");
	yarp.connect("/aber/reach/status:o", "/smc/reaching/in");

	tor->initMatlabPorts();

	yarp::os::Time::delay(0.01);
	write(neu);

	while(true) {

		yarp::os::Bottle request, reply;
		port.read(request, true);
		if(request.get(0).asString() == "get_sensors") {
			cout << " ***** " << "received get_sensors command" << " ***** " << endl;
			add_visual_targets(&reply);
			add_touches(&reply);
		} else if(request.get(0).asString() == "get_reaches") {
			cout << " ***** " << "received get_reaches command" << " ***** " << endl;
			get_known_reaches(&reply);
		} else if(request.get(0).asString() == "reach") {
			double x = request.get(1).asDouble();
			double y = request.get(2).asDouble();
			double z = request.get(3).asDouble();
			int arm = request.get(4).asInt();
			printf(" ***** received command to reach to (%.2f, %.2f, %.2f) with arm %f ***** \n",x,y,z,arm);
//			cout << "received command to reach to (" << x << ", " << y << ")" << endl;
			reach_to_target(&reply, x, y, z, arm, true);
		} else if(request.get(0).asString() == "point") {
			double x = request.get(1).asDouble();
			double y = request.get(2).asDouble();
			double z = request.get(3).asDouble();
			int arm = request.get(4).asInt();
			printf(" ***** received command to point to (%.2f, %.2f, %.2f) with arm %f ***** \n",x,y,z,arm);
//			cout << "received command to reach to (" << x << ", " << y << ")" << endl;
			reach_to_target(&reply, x, y,z,arm, true);
		} else if(request.get(0).asString() == "press") {
			cout << " ***** " << "received press command" << " ***** " << endl;
			press(&reply);
		} else if(request.get(0).asString() == "saccade") {
			double x = request.get(1).asDouble();
			double y = request.get(2).asDouble();
			double z = request.get(3).asDouble();
//			int destination = request.get(1).asInt();
			cout << " ***** received command to saccade to ("<< x << ", " << y  << ", " << z<< ") ***** " <<endl;
			saccade(&reply, x,y,z);
		} else if(request.get(0).asString() == "grasp") {
			cout  << " ***** " << "received grasp command" << " ***** " << endl;
			grasp(&reply, 0);
		} else if(request.get(0).asString() == "release") {
			cout << " ***** " << "received release command" << " ***** " << endl;
			grasp(&reply, 1);
		} else if(request.get(0).asString() == "get_speech_excitation") {
			cout << " ***** " << "received get_speech_excitation command" << " ***** " << endl;
			const char *word = request.get(1).asString().c_str();
			get_speech_excitation(&reply, word);
		} else {
			reply.addString("Unknown command");
		}

		port.reply(reply);
	}
}

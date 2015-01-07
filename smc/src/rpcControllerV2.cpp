/*
 * rpcController.cpp
 *
 * Provides an RPC interface to the SMc functionality
 * Adapted to function with the new and improved reaching
 *
 *  Created on: 10/03/2012
 *  Author: Michael Sheldon <mts08@aber.ac.uk>
 */

#include <yarp/os/all.h>

#include <signal.h>
#include "armReaching.h"
#include "EyeHeadController.h"
#include "torsoSaccading.h"
#include "handEyeCoordination.h"
#include "tracker.h"
#include "graspController.h"
#include "TorsoReach.h"

#include "ShortTermMemoryV2.h"

#define OCCLUSION_THRESHOLD 0.5
#define GRASP_DISTANCE_THRESHOLD 0.1
#define MAX_OBJECTS 5
#define HAND_ID 2
#define NEAREST_REACH_RADIUS 2

armReaching* armReach;
EyeHeadSaccading* ehCont;
handEyeCoordination* heCoor;
graspController *grippy;
torsoSaccading* tor;
TorsoReach* torRoar;
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

/*
 * In experiments for sequential vs synchronous learning, these variable should be changed!
 */
//bool synchronous = false;
//bool nearestNeighbour = true;

yarp::os::BufferedPort<yarp::os::Bottle> tport;

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


void saveAndQuit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
	heCoor->closeLogs();

	ehCont->saveMaps();
	tor->saveMapping();
	heCoor->saveGazeReachMap();

	std::exit(param);
}


void quit(int param)
{
	target->closeLog();
	ehCont->closeLogs();
	tor->closeLogs();
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


bool nearest_reach_for_gaze(GazeField *gf, ReachField *rf) {
	double correlation;
	double *headMotorConfig = new double[6];
	HeadConfig *hc = gf->getPreferredGaze();
	hc->toArray(headMotorConfig);
	double dist;
	bool found = heCoor->getNearestReachForGaze(gf, &rf, &dist);	//gm->getNearestReachFieldForGaze(hc, rf, &correlation, NEAREST_REACH_RADIUS);
	return found;
}


void get_known_reaches(yarp::os::Bottle *reply) {
	vector<GazeReachLink *> fields = gm->getLinks();
//	vector<GazeReachLink *> fields = heCoor->getTableReaches();	//gm->getGazeFields();
	for(int i = 0; i < fields.size(); i++) {
		GazeReachLink* l = fields.at(i);
		GazeField *gf = l->gaze;
		ReachField *rf = l->reach;
//		bool found = nearest_reach_for_gaze(gf, rf);
//		if(found) {
			double gazeX, gazeY;
//			gf->getGazeDirection(&gazeX, &gazeY);
			reply->addDouble(gf->getXcoord());
			reply->addDouble(gf->getYcoord());
//		}
	}
}


void reach_to_target(yarp::os::Bottle *reply, double x, double y) {
	GazeField *gf = gm->getGazeField(x, y);
	yarp::os::Time::delay(0.01);
	write(neu);
	yarp::os::Time::delay(0.01);
	if(gf->getXcoord()==0 && gf->getYcoord()==0)
	{
		double dist;
		gm->getNearestGazeField(x, y, &gf, &dist);
		printf("Had to get nearest field, at a distance of %.2f", dist);
	}
	cout << "obtained gaze field: " << *gf << endl;

	ReachField *rf = new ReachField();
	bool found;
	double dist;
	rf = heCoor->getLinkedReach(gf);	//gm->getHandCoordination(gf, rf);
	found = rf->getXcoord()!=0 && rf->getYcoord()!=0;
	if(!found) {
		found = heCoor->getNearestReachForGaze(gf, &rf, &dist);	//nearest_reach_for_gaze(gf, rf);
		if(!found) {
			reply->addString("FAILED");
			yarp::os::Time::delay(0.01);
			write(sad);
			yarp::os::Time::delay(0.01);
			return;
		}
	}
	cout << "obtained reach field: " << *rf << endl;
	ehCont->goToGazeField(gf);
	// Update our memory of where we last saw the hand
	//prevFields[HAND_ID] = gf;
	stm->updateHandPosition(gf);

	//look to see if there is anything in that direction
//	double targX, targY, depth;
//	string col = target->getNearestObject(&targX, &targY, &dist);
//	if(dist>0 && dist<32)
//	{
//		//Objects visible and possibly within fovea, or almost in fovea
//		bool success = ehCont->getDepth(col, &depth);
//		heCoor->getNearestReachForGaze(gf, &rf, depth, &dist);
//		if(!(dist>0 && dist<NEAREST_REACH_RADIUS))
//		{
//			success = torRoar->getReach(targX, targY, depth, col, 5);
//		}
//	}
//	else
		armReach->goToReachField(rf); 	//reachToField(rf, true);

	do{
		yarp::os::Time::delay(0.2);
	}while(!heCoor->stationary());		//Arm goes to rest
	yarp::os::Time::delay(0.5);
	do{
		yarp::os::Time::delay(0.2);
	}while(!heCoor->stationary());		//Arm goes to new position

	reply->addString("SUCCESS");
	yarp::os::Time::delay(0.01);
	write(hap1);
}


void press(yarp::os::Bottle *reply) {
	armReach->getArmController()->pressButton(rightArm);
	reply->addString("SUCCESS");
	yarp::os::Time::delay(0.01);
	write(hap2);
}


void saccade(yarp::os::Bottle *reply, double x, double y) {// int destinationField) {
	GazeField *gf = gm->getGazeField(x, y);	//gm->getGazeFields().at(destinationField);
	ehCont->goToGazeField(gf);
	reply->addString("SUCCESS");
	yarp::os::Time::delay(0.01);
	write(hap1);
}


void grasp(yarp::os::Bottle *reply, bool release) {
#ifdef SIMULATOR
	Network yarp;
	RpcClient rpcPort;
	rpcPort.open("/smc/grasp");
	yarp.connect("/smc/grasp", "/icubSim/world");
	if(!release) {
		// Find hand position
		Bottle hcmd, hresponse;
		hcmd.addString("world");
		hcmd.addString("get");
		hcmd.addString("rhand");
		rpcPort.write(hcmd, hresponse);
		double hx = hresponse.get(0).asDouble();
		double hy = hresponse.get(1).asDouble();
		double hz = hresponse.get(2).asDouble();
		// Find object positions
		double nx, ny, nz;
		const char *nearestType = NULL;
		int nearestObject = -1;
		double nearestDistance = GRASP_DISTANCE_THRESHOLD;
		const char *type;
		const char *types[] = {"box", "sph", "cyl"};
		for(int t = 0; t < 3; t++) {
			type = types[t];
			for(int o = 1; o <= MAX_OBJECTS; o++) {
				Bottle ocmd, oresponse;
				ocmd.addString("world");
				ocmd.addString("get");
				ocmd.addString(type);
				ocmd.addInt(o);
				rpcPort.write(ocmd, oresponse);
				double ox = oresponse.get(0).asDouble();
				double oy = oresponse.get(1).asDouble();
				double oz = oresponse.get(2).asDouble();
				double distance = sqrt(pow(hx - ox, 2) + pow(hy - oy, 2) + pow(hz - oz, 2));
				if (distance < nearestDistance) {
					nearestDistance = distance;
					nearestObject = o;
					nearestType = type;
					nx = ox;
					ny = oy;
					nz = oz;
				}
			}
		}
		if (nearestObject != -1) {
			// Move nearest object to hand
			holdingType = nearestType;
			holdingObject = nearestObject;
			Bottle mcmd, mresponse;
			mcmd.addString("world");
			mcmd.addString("set");
			mcmd.addString(holdingType);
			mcmd.addInt(holdingObject);
			mcmd.addDouble(nx);
			mcmd.addDouble(hy - 0.01);
			mcmd.addDouble(nz);
			rpcPort.write(mcmd, mresponse);

			// Assume the object we gripped was in the same field
			for (int i = 1; i < 6; i++) {
				if (i != 2 && objects[i] == objects[2]) {
					holdingObjectId = i;
				}
			}
		} else {
			// We're 'holding' our own hand
			holdingObjectId = HAND_ID;
			reply->addString("FAILED");
			return;
		}
	} else {
		holdingObjectId = -1;
	}

	// Attach or release the object
	Bottle cmd, response;
	cmd.addString("world");
	cmd.addString("grab");
	cmd.addString(holdingType);
	cmd.addInt(holdingObject);
	cmd.addString("right");
	cmd.addInt((int) !release);
	rpcPort.write(cmd, response);
	reply->addString("SUCCESS");
#else
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
#endif
}


void get_speech_excitation(yarp::os::Bottle *reply, const char *word) {
	reply->addDouble(1.0);
}


int main(int argc, char* argv[])
{
	srand(NULL);

	yarp::os::Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	yarp::os::Value* val;
	std::string robot;
	if(options.check("robot",val))
	{
		robot = val->asString().c_str();
		cout << "Selected robot: " << robot << endl;
	}
	else
	{
		cout << "A robot can be specified from the command line e.g. --robot [icub|icubSim]+F" << endl;
		robot = "icub";
	}

	yarp::os::Network::init();
	tport.open("/smc/target_read");
	target = new Target();
	bool load = true;
	string path = "../data/";
	string filename = "testXV10";
	target->initLog(path);
	heCoor = new handEyeCoordination(load, path, filename);
	gm = heCoor->getGazeMap();
	ehCont = new EyeHeadSaccading(robot, gm, target, SYNCHRONOUS, NEAREST_NEIGHBOUR, load, path, filename, true);
	tor = new torsoSaccading(robot, ehCont, target, true, NEAREST_NEIGHBOUR, path, load, filename);
	armReach = new armReaching(robot, gm, true, true);
	grippy = new graspController(robot, armReach->getArmController());

	printf("Initiating short term memory...\n");
	stm = new ShortTermMemory(target, ehCont);//, tor);
	signal(SIGINT, saveAndQuit);
	heCoor->init(ehCont, armReach, target, true);
	torRoar = new TorsoReach(tor, heCoor, ehCont);

	printf("Opening rpc port...\n");
//	RpcServer port;
	yarp::os::RpcServer port;
	port.open("/smc/rpc");
	cout << "Server ready." << endl;


	printf("Opening emote port...\n");
	emotePort.open("/emote/ask");
//	yarp::os::Network::connect("/emote/ask", "/emote/emotion");

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
			printf(" ***** received command to reach to (%.2f, %.2f) ***** \n",x,y);
//			cout << "received command to reach to (" << x << ", " << y << ")" << endl;
			reach_to_target(&reply, x, y);
		} else if(request.get(0).asString() == "point") {
			double x = request.get(1).asDouble();
			double y = request.get(2).asDouble();
			printf(" ***** received command to point to (%.2f, %.2f) ***** \n",x,y);
//			cout << "received command to reach to (" << x << ", " << y << ")" << endl;
			reach_to_target(&reply, x, y);
		} else if(request.get(0).asString() == "press") {
			cout << " ***** " << "received press command" << " ***** " << endl;
			press(&reply);
		} else if(request.get(0).asString() == "saccade") {
			double x = request.get(1).asDouble();
			double y = request.get(2).asDouble();
//			int destination = request.get(1).asInt();
			cout << " ***** received command to saccade to ("<< x << ", " << y << ") ***** " <<endl;
			saccade(&reply, x,y);
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

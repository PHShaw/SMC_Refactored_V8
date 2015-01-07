/*
 * ShortTermMemory.h
 *
 * Question: should this be running as a thread in the background?
 *
 *  Created on: 27 Apr 2012
 *      Author: icub
 */

#ifndef SHORTTERMMEMORYV2_H_
#define SHORTTERMMEMORYV2_H_

#include <iostream>
#include <math.h>
#include <vector>

#include "CalcGazeChange.h"
#include "EyeHeadController.h"
//#include "torsoSaccading.h"
#include "Target.h"



struct object{

	object(string desc, double d, int s, int pID=1)
	{
		id = pID;
		description = desc;

		depth = (double)((int)( d*100))/100;
		size  = s;
		moved = false;
		disappeared = false;
		report = false;
		excitation = INITIAL_EXCITATION;
		stacked = false;
		occluded = false;
		fixateCount = 1;
	}

	void update(double nd, int ns, bool mov)
	{
		moved = mov;
		depth = (double)((int)( nd*100))/100;
		size  = ns;
		excitation = INITIAL_EXCITATION;
	}

	bool gotFeatures()
	{
		return VAM_featureLocations.size()>0;
	}

	int id;
	string description;			//colour
	double depth;
	int size;
	int excitation;
	bool moved, disappeared,report;
	bool stacked;
	bool occluded;

	int fixateCount;

	std::vector<std::pair<int, int> > VAM_featureLocations;
	std::vector<std::vector<float> > VAM_featureVectors;	//dimensions: featureCount x 40

	const static int INITIAL_EXCITATION = 10;

};

std::ostream& operator <<(std::ostream& outs, const object& source);

/**
 * Equivalence based only on description, not position
 */
bool operator ==(const object& f1, const object& f2);

/**
 * Constant comparator, based on description rather than variable position or excitation
 */
bool operator <(const object& o1, const object& o2);

/**
 * Variable comparator, based on excitation for dynamic sorting of interesting objects
 */
bool compare(object* o1, object* o2);



class ShortTermMemory
{
public:
//	ShortTermMemory(Target* t, EyeHeadController* eh, torsoSaccading* ts);
	ShortTermMemory(Target* t, EyeHeadSaccading* eh);
	virtual ~ShortTermMemory();


	object* getTarget();	//returns object with highest excitation
	void update(int holdingObjectID=-1);
	void update(string description, GazeField* gf);

	void checkForStacks();		//TODO: not implemented and probably need to return something here?

	void printMemory();

	void reportMemory(yarp::os::Bottle *reply, int holdingObjectID=-1);
	void getVisualTouch(yarp::os::Bottle *reply);

	void updateHandPosition(GazeField* gf);
	GazeField* getHandGaze();
	int getNewHoldingID();
	int size(){return objectToFieldMemory.size();}
	std::multimap<object*, GazeField*> getObjectMemory(){return objectToFieldMemory;}
	object* getObject(std::string description);

private:

	Target* target;
	EyeHeadSaccading* ehCont;
//	torsoSaccading* torSac;


	bool fixate(object* o, GazeField** gf);
	bool reFixate(object* o, GazeField* gf_old, GazeField** gf_new, double* dist);
	void changeLink(object* o, GazeField* gf_old, GazeField* gf_new);	//TODO: not yet implemented
	void paint(object* o, GazeField* gf);
	void repaint(object* o, GazeField* gf_old, GazeField* gf_new, int newSize);
	int colourToId(const char *colour);
	string idToColour(const int id);

	//These two should be kept synchronised to ease finding things in both directions.  //TODO is there a better way of doing this?
	std::multimap<GazeField*, object*> fieldToObjectMemory;
	std::multimap<object*, GazeField*> objectToFieldMemory;

	int reachDepthThreshold;

	//Optional extra to display gaze fields that are currently remembered
	yarp::os::BufferedPort<yarp::os::Bottle> gazed;
	const static double OCCLUSION_THRESHOLD = 0.5;
	const static double MOVEMENT_THRESHOLD = 2.5;

	unsigned int objIDcounter_smc;

};

#endif /* SHORTTERMMEMORY_H_ */

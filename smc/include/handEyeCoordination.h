/*
 * handEyeCoordination.h
 *	used for linking the gaze map and reach map together
 *
 *
 *  TODO: Introduce logging!!!
 *
 *  Created on: 17 Aug 2011
 *      Author: icub
 */

#ifndef HANDEYECOORDINATION_H_
#define HANDEYECOORDINATION_H_

#include "armReaching.h"
#include "EyeHeadController.h"
#include "Gaze_IO.h"

class handEyeCoordination
{
public:
	handEyeCoordination(bool ploadFile, string path="../data/", string filename="testXV10");
	virtual ~handEyeCoordination();

	void init(EyeHeadSaccading* ehCont, armReaching* ar, Target* target, bool learn);


	bool findArm(ReachField* reach, bool rightArm);	//Visually guided reaching
	bool handGaze(bool rightArm);
	bool checkArmLinks(ReachField* reach);

	bool handCentred();
	string handVisible(double* targX, double* targY, bool* success);
	int learnCoordination(bool rightArm);

	bool addLink(GazeField* gf, ReachField* rf, double depth);
	bool updateLink(ReachField* rf, GazeField* newGF, double depth);

	void reachTo(ReachField* reach);
	void armRest(){ar->toRest();}
	bool stationary();


	GazeMap* getGazeMap();
	bool saveGazeReachMap();
	bool loadGazeReachMap(string filename);


//	bool isReachField(double x, double y);
//	ReachField* getReachField(double x, double y);
//	bool isGazeField(double x, double y);
//	GazeField* getGazeField(double x, double y);
//	bool isLinkedInput(PolarField* motorField);
//	PolarField* getLinkedInput(PolarField* motorField);
//	bool isLinkedOutput(PolarField* retinaField);
//	PolarField* getLinkedOutput(PolarField* retinaField);


	armController* getArmController(){return ar->getArmController();}
	armReaching* getArmReaching(){return ar;}
	EyeHeadSaccading* getEyeHeadController(){return ehCont;}


	bool hasLinkedReach(GazeField* gaze, double depth);
	bool hasLinkedGaze(ReachField* reach);
	ReachField* getLinkedReach(GazeField* gaze, double depth);
	ReachField* getLinkedReach(GazeField* gaze);
	GazeField* getLinkedGaze(ReachField* reach);


	bool getNearestReachForGaze(GazeField* gf, ReachField** rf, double* dist);
	bool getNearestReachForGaze(GazeField* gf, ReachField** rf, double depth, double* dist);
	bool getNearestLinkForGaze(GazeField* gf, GazeReachLink** link, double depth, double* dist);

	std::vector<GazeReachLink*> getTableReaches();


	void initLogs();
	void logEntry(string message);
	void logField(const Field* field);
	void startCoordLog(int counter);
	void closeLogs();
private:
	EyeHeadSaccading* ehCont;
	armReaching* ar;
	Target* target;
	GazeMap* gm;

	bool learn;

	string path, filename;
	bool loadFile;

	ofstream handEyelog;
};

#endif /* HANDEYECOORDINATION_H_ */

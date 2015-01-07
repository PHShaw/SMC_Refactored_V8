/*
 * torsoSaccading.h
 *
 *  Created on: 8 Jun 2011
 *      Author: icub
 */

#ifndef TORSOSACCADING_H_
	#define TORSOSACCADING_H_


#include <fstream>
#include <iostream>
#include <string.h>
#include <math.h>

#include "EyeHeadController.h"
#include "torsoController.h"
#include "FieldFieldMapping.h"	//typedef'ed as ffm?
#include "Target.h"
#include "torsoCompensator.h"
#include "tracker.h"

#include "GazeMap.h"

typedef FieldFieldMapping ffm;



class torsoSaccading
{
public:
	torsoSaccading(EyeHeadSaccading* pEhcont, Target* pTarget);
	virtual ~torsoSaccading();

	bool loadMapping(std::string filename);
	bool saveMapping();


	bool allStationary();
	void recordPrePositions();
	void toRest(){torso->move(0,0,true);}

	void compensateToRest();

	bool gotoField(Field* motorfield, bool compensate=false);
//	void makeRandomMove();
	bool makeLink(double startX, double startY, double endX, double endY, double motorRelX, double motorRelY);
	bool makeLink(double startX, double startY, double endX, double endY, PolarField* motor);
	bool makeLink(double inputX, double inputY, double motorRelX, double motorRelY);
	bool makeLink(PolarField* source, PolarField* motor);
//	int randGenerator(int range);

	bool calcLink(std::string);
	int learnMapping(string colour);
	int learnChain();
	bool saccade(int saccadecounter, std::string colour);


//	void startCompensator();
//	void stopCompensator();


	bool easeHead();

	//No learning, single movement saccade - no guarantee of fixation
	//bool simpleSaccade();

	bool followLink(double relX, double relY);

	//Map based methods
//	bool getRetinaField(PolarField* field, double x, double y);
	bool isRetinaField(double x, double y);
	PolarField* getRetinaField(double x, double y);
//	bool getMotorField(PolarField* field, double x, double y);
	bool isMotorField(double x, double y);
	PolarField* getMotorField(double x, double y);

	bool isLinkedInput(PolarField* motorField);
	PolarField* getLinkedInput(PolarField* motorField);
	bool isLinkedOutput(PolarField* retinaField);
	PolarField* getLinkedOutput(PolarField* retinaField);

	PolarField* getNearestLearntInput(double targX, double targY, float* dist);
	PolarField* getNearestLearntOutput(double x, double y, float* dist);



//**************************************************
// LWPR functions
	void initMatlabPorts();
	void LWPR_TorsoLearner(bool eyeAndHead=true);
	void LWPR_TorsoLearnerFullModel();
	void LWPR_Tester(std::string colour, bool eyeAndHead=true);
	void LWPR_TorsoReach(string colour, bool rightArm);
	void LWPR_TorsoReach(double desiredX, double desiredY, double desiredVerg);
	void LWPR_centerTorsoOnTarget(double vergence);

	void closeMatlabPorts();

	yarp::os::BufferedPort<yarp::os::Bottle> portOut;
	yarp::os::BufferedPort<yarp::os::Bottle> portIn;
	yarp::os::Bottle* input;
	ofstream LWPR_Logfile;
	ofstream LWPR_FullLogfile;

//**************************************************


	void openLogs();
	void motorRecord(double relX, double relY, int saccadeCounter);
	void logEntry(string message);
	void logEntry(string message, double x, double y);
	void logField(const Field* field);
	void startSaccadeLog(int saccadeCounter);
	void endSaccadeLog(bool success);
	void closeLogs();



	//botch functions
	void initCompensator()
	{
		yarp::os::Network yarp;
		yarp::dev::PolyDriver* headMotor = ehCont->getMotorDriver();
		yarp::dev::PolyDriver* torsoMotor = torso->getMotorDriver();
		tc->initTorsoCompensator(torsoMotor, headMotor);
	}

	torsoController* getTorsoController(){return torso;}

private:



	torsoController* torso;
	EyeHeadSaccading* ehCont;
	torsoCompensator* tc;

	//boost::thread tcThrd;

	ffm* torso_ppm;

	Target* target;



	double gazeX1, gazeY1;	//pre and post eye saccade
	double torsoX1, torsoY1;

	int saccadeCounter;

	headSaccading* headSac;

	std::ofstream motorlogfile;
	std::ofstream logfile;

	std::list<position> torsoChain;
};

#endif /* TORSOSACCADING_H_ */

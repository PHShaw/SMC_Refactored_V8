/*
 * eyeSaccading.h
 *
 *  Created on: 6 Jun 2011
 *      Author: icub
 */

#ifndef EYE_SACCADING
	#define EYE_SACCADING

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <string.h>


#include "eyeController.h"
#include "FieldFieldMapping.h"	//typedef'ed as ffm?
#include "Target.h"
#include "VamTarget.h"

typedef FieldFieldMapping ffm;



struct position{
	position()
	{
		eyeMotorX = 0;
		eyeMotorY = 0;
		targetX = 0;
		targetY = 0;
	}
	position(double eX, double eY, double tX, double tY)
	{
		eyeMotorX = eX;
		eyeMotorY = eY;
		targetX = tX;
		targetY = tY;
	}
	double eyeMotorX;
	double eyeMotorY;
	double targetX;
	double targetY;
};


struct relativeMovement{
	relativeMovement(double startX, double startY)
	{
		current.eyeMotorX = startX;
		current.eyeMotorY = startY;
		relativeX = 0;
		relativeY = 0;
	}

	relativeMovement operator+= (position p)	//Build up a relative movement working forwards
	{
		double xChange = p.eyeMotorX - current.eyeMotorX;
		double yChange = p.eyeMotorY - current.eyeMotorY;

		relativeX += xChange;
		relativeY += yChange;

		current.eyeMotorX = p.eyeMotorX;
		current.eyeMotorY = p.eyeMotorY;
	}

	relativeMovement operator-= (position p)	//Build up a relative movement working backwards
	{
		double xChange = current.eyeMotorX - p.eyeMotorX;
		double yChange = current.eyeMotorY - p.eyeMotorY;

		relativeX += xChange;
		relativeY += yChange;

		current.eyeMotorX = p.eyeMotorX;
		current.eyeMotorY = p.eyeMotorY;
	}

	double relativeX;
	double relativeY;

private:
	position current;
};


struct FailureReport{

	void reset()
	{
		startX = startY = endX = endY = 0;
		nearestNeighbour = false;
		dist = 0;
		startInput = NULL;
		nearestInput = NULL;
		output = NULL;

		failedToObtainedCurrentEyePosition = false;
		unreachable = false;
		combiSaccade = false;
	}
	double startX, startY, endX, endY;
	bool nearestNeighbour;
	float dist;
	PolarField* startInput;
	PolarField* nearestInput;	//Note, may be the same as start Input, if was unable to locate retina field
	PolarField* output;

	bool failedToObtainedCurrentEyePosition;
	bool unreachable;	//output motor movements went beyond motor range

	bool combiSaccade;	//combined eye and head movement used
};



class eyeSaccading{
public:
	eyeSaccading(eyeController* pEye, Target* pTarget, ffm* eye_ppm, bool learn, bool pNeigh, std::string path);
	~eyeSaccading();


	//Methods taken from original eye saccading, may require some redesign
//	void makeRandomMove();
	bool gotoField(Field* motorfield);
	bool followLink(double relX, double relY);

	bool saccade(int saccadecounter, double targX, double targY);
	bool saccade(int saccadecounter, double targX, double targY, std::string colour);
		//This method will need to be segmented!

	//No learning, single movement saccade - no guarantee of fixation
	bool simpleSaccade(double targX, double targY, std::string colour, bool check=true);

	//Map based methods
//	bool getRetinaField(PolarField* field, double x, double y);
	bool isRetinaField(double x, double y);
	PolarField* getRetinaField(double x, double y);
	bool isMotorField(double x, double y);
	PolarField* getMotorField(double x, double y);
	bool isLinkedInput(PolarField* motorField);
	PolarField* getLinkedInput(PolarField* motorField);
	bool isLinkedOutput(PolarField* retinaField);
	PolarField* getLinkedOutput(PolarField* retinaField);
	FieldLink* getLinkFromOutput(PolarField* motorField);
	FieldLink* getLink(PolarField* input, PolarField* output){return ppm->getLink(input, output);}

	PolarField* getNearestLearntInput(double targX, double targY, float* dist);
	PolarField* getNearestLearntOutput(double x, double y, float* dist);
	PolarField* getNearestReliableLearntInput(double targX, double targY, float* dist);
	PolarField* getNearestReliableLearntOutput(double x, double y, float* dist);


	bool makeLink(double startX, double startY, double endX, double endY, double motorRelX, double motorRelY, int calcdLinks=100);
	bool makeLink(double startX, double startY, double endX, double endY, PolarField* motor, int calcdLinks=100);
	bool makeLink(double sourceX, double sourceY, double motorRelX, double motorRelY, int calcdLinks=100);
	bool makeLink(PolarField* source, PolarField* motor, int calcdLinks=100);

	int learnChain();

	void stopLearning();
	void startLearning();


	bool getDepth(std::string colour, double* depth);	//measured in mm

	bool autoCenter(double targX, double targY, std::string colour, bool check=true);		//expected to be used with the target already centred in the fovea

	//Logging and stats methods:
	void openLogs();
	void motorRecord(double relX, double relY);
	void logEntry(std::string message);
	void logEntry(std::string message,double x,double y);
	void logField(const Field* field);
	void startSaccadeLog();
	void endSaccadeLog(bool success);
	void closeLogs();

	void resetStats();
	int getStepCounter(){return stepCounter;}
	void getStats(int* saccadecounter, int* stepcounter, bool* successfulDirectLink, bool* successfulNeighbourLink,
			int* unsuccesfulDirectLinkCounter, int* neighbourCounter, bool* penultimateMoveWasLink,
			int* linksLearnt, int* linksUpdated, int* possibleLinkstoLearn);

	void incrementSaccadeCounter(){saccadeCounter++;}

	bool verge(std::string colour);
	bool verge(VamTarget* vam);
	bool verge(double lx, double ly, double rx, double ry);


	FailureReport fr;
private:

//	bool getCurrentPosition(double* x, double* y);


	eyeController* eye;
	ffm* ppm;
	Target* target;

	std::list<position> saccadeSteps;

	bool learn;
	bool nearestNeighbour;


	//Stats collection
	int saccadeCounter;
	int stepCounter;
	bool successfulDirectLink;
	bool successfulNeighbourLink;
	int unsuccesfulDirectLinkCounter;
	int neighbourCounter;
	bool penultimateMoveWasLink;
	int linksLearnt;
	int linksUpdated;
	int possibleLinkstoLearn;

	std::string path;
	std::ofstream motorlogfile;
	std::ofstream logfile;
	std::ofstream eyeLinkLog;
};




#endif

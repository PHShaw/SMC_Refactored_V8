/*
 * headSaccading.h
 *
 *  Created on: 8 Jun 2011
 *      Author: icub
 */

#ifndef HEADSACCADING_H_
	#define HEADSACCADING_H_


#include <fstream>
#include <iostream>
#include <string.h>

#include "eyeController.h"
#include "eyeSaccading.h"
#include "headController.h"
#include "FieldFieldMapping.h"	//typedef'ed as ffm?
#include "Target.h"
#include "vor2.h"

typedef FieldFieldMapping ffm;

class headSaccading
{
public:
	headSaccading(headController* pHead, eyeSaccading* pEyeSac, Target* pTarget, ffm* head_ppm);
	virtual ~headSaccading();

	void reloadMaps( ffm* ppm){head_ppm=ppm;}

	bool allStationary();
	void recordPrePositions();

	bool gotoField(Field* motorfield);
//	void makeRandomMove();
	bool makeLink(double startX, double startY, double endX, double endY, double motorRelX, double motorRelY);
	bool makeLink(double startX, double startY, double endX, double endY, PolarField* motor);
	bool makeLink(double inputX, double inputY, double motorRelX, double motorRelY);
	bool makeLink(PolarField* source, PolarField* motor);
//	int randGenerator(int range);

	bool calcLink(std::string, vor* v);
	bool addFullHeadMovement();
	bool saccade(int saccadecounter, double pstartTargX, double pstartTargY, std::string colour, vor* v);

	bool iStyleSaccade(int saccadeCounter, double targX, double targY, std::string colour);
	int learnChain();

	//No learning, single movement saccade - no guarantee of fixation
	bool simpleSaccade();

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

	void openLogs();
	void motorRecord(double relX, double relY, int saccadeCounter);
	void logEntry(std::string message);
	void logEntry(std::string message, double x, double y);
	void logField(const Field* field);
	void startSaccadeLog(int saccadeCounter);
	void endSaccadeLog(bool success);
	void closeLogs();

private:


	headController* head;
	eyeController* eye;

	eyeSaccading* eyeSac;

	ffm* head_ppm;

	Target* target;
	double targX, targY;
	double startTargX, startTargY;


	double EyeX1, EyeX2, EyeY1, EyeY2;	//pre and post eye saccade
	double HeadX1, HeadY1;


	int saccadeCounter;

	//For iStyleSaccade
	std::list<position> saccadeSteps;
	//--------------------

	std::ofstream motorlogfile;
	std::ofstream logfile;
};

#endif /* HEADSACCADING_H_ */

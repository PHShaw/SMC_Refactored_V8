/*
 * armReaching.h
 *
 *  Created on: 17 Aug 2011
 *      Author: icub
 */

#ifndef ARMREACHING_H_
#define ARMREACHING_H_

#include <string.h>
#include <iostream>
#include <fstream>

#include "GazeMap.h"
#include "armController.h"

class armReaching
{
public:
	armReaching(GazeMap* pgm);
	virtual ~armReaching();

	bool getReachField(double depth, double angle, ReachField** reach);
	bool getReachField(ReachField** reach, bool rightArm);	//returns reach fields for current position,
											//guarantees a proper reach field, but may be newly created

	//Current coordinates
	void getCoordinates(double* depth, double* angle, bool rightArm);

//	void goToReachField(double depth, double angle);
	void goToReachField(ReachField* reach);
	bool isReachField(bool rightArm);
	ReachField* getReachField(bool rightArm);

	bool addReachField(bool rightArm);

	void goToReachConfig(ReachConfig* rc);

	void toRest();
	void toRest(bool rightArm);

	bool learnReach(int rightArm);		//can also be called from handEye to move arm to new location and link gaze
	int learnReaching(int rightArm);	//repeatedly call learn reach, also time it, and return no. reaches!
	int learnReaching(bool rightArm);				// need to identify some kind of threshold to decide when learnt
		//randomly select which arm		// also, remember the reaching can be for one or both arms.



	//Logging methods:
	void openLogs();
	void logCoords(double depth, double angle);
	void logEntry(std::string message);
	//void logEntry(std::string message,double x,double y);
	void logField(const Field* field);
	void logPosition(const double* position);
	void startReachLog(int counter);
	void endReachLog(bool success);
	void closeLogs();


	armController* getArmController(){return armCont;}

private:
	armController* armCont;
	GazeMap* gm;


	std::ofstream logfile;

	//Learn arm mapping based on proprioception.
	//link arm reaching with gazing.
};

#endif /* ARMREACHING_H_ */

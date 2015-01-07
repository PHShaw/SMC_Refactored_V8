/*
 * TorsoReach.h
 *
 *  Created on: 30 May 2012
 *      Author: icub
 */

#ifndef TORSOREACH_H_
#define TORSOREACH_H_

#include "EyeHeadController.h"
#include "handEyeCoordination.h"
#include "torsoSaccading.h"
#include "utilities.h"

class TorsoReach
{
public:
	TorsoReach(torsoSaccading* t, handEyeCoordination* h, EyeHeadSaccading* ehc);
	virtual ~TorsoReach();

	bool gazeReach(string colour, double range=1);
	bool getReach(double targetGazeX, double targetGazeY, double targetDepth, string colour, double range=1);
	bool search(float x, float y);

private:

	void points();

//	EyeHeadController* ehCont;
	GazeMap* gm;
	handEyeCoordination* heCoor;
	torsoSaccading* tor;
	EyeHeadSaccading* ehCont;


	std::vector<float> torsoPoints;	//Only x points for torso motor map, y=0, working from centre out, alternating sides
};


#endif /* TORSOREACH_H_ */

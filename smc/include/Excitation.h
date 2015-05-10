/*
 * Excitation.h
 *
 * A set of excitation levels for different subsystems.  The values should be in the
 * range 0-1.
 *
 *  Created on: 4 May 2015
 *      Author: icub
 */

#ifndef EXCITATION_H_
#define EXCITATION_H_

#include <iostream>
#include <map>
#include <cmath>
#include <string.h>

#include "params_config.h"

namespace smc
{

enum System {RETINA, FOVEAL, ARM, HAND, EYE};

class Excitation
{
public:
	Excitation(float initialExcitation);
	virtual ~Excitation();

	void updateGlobalExcitation();
	float getGlobalExcitation(){return globalExcitation;}

	System getMaxExcitation();
	void setEyeExcitation(int mapSaturation);	//Indication of learning completeness
	void updateEyeExcitation(int steps, int fieldsLearnt); //update as learning
	void updateEyeExcitation(int steps, bool success);		//updates the EYE value, when not learning

	void setFovealExcitation(int acuity, int fov);
	void updateFovealExcitation(int acuity, int fov);	//updates the FOVEAL value

	void setRetinaExcitation();
	void updateRetinaExcitation(int visualTargets=0);		//updates the RETINA value (using the flags from params_config.h)
	void updateRetinaExcitation(int colourTargets, int motionTargets);

	void setReachExcitation(int stage, float mapSaturation);
	void updateReachExcitation(float distance);		//updates the ARM and HAND value

	float getExcitation(System system);


	void decay(System subsystem);
	void stimulateSystem();
	void stimulateSubSystem(System subsystem);


	void printExcitations();
//	TODO: print and log excitation levels.
private:
	float globalExcitation;
	std::map<System, float> subsystems;

	unsigned char currentVisionFlags;
};

} /* namespace smc */
#endif /* EXCITATION_H_ */

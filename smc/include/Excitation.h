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
	void updateEyeExcitation(int steps);		//updates the EYE value, when not learning

	void setFovealExcitation(int acuity, int fov);
	void updateFovealExcitation(int acuity, int fov);	//updates the FOVEAL value

	void setRetinaExcitation();
	void updateRetinaExcitation();		//updates the RETINA value (using the flags from params_config.h)

	void setReachExcitation(int stage, float mapSaturation);
	void updateReachExcitation(float distance);		//updates the ARM and HAND value


	void decay(System subsystem);

//	TODO: print and log excitation levels.
private:
	float globalExcitation;
	std::map<System, float> subsystems;

	unsigned char currentVisionFlags;
};

} /* namespace smc */
#endif /* EXCITATION_H_ */

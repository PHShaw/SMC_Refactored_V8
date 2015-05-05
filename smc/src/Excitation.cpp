/*
 * Excitation.cpp
 *
 *  Created on: 4 May 2015
 *      Author: icub
 */

#include "Excitation.h"
#include "params_config.h"

using namespace std;

namespace smc
{

Excitation::Excitation(float initialExcitation)
{
	subsystems[RETINA] = initialExcitation;		//changes on the retina, e.g. phasic stimuli
	subsystems[FOVEAL] = initialExcitation;		//foveal maturity, e.g. image detail
	subsystems[ARM]	   = initialExcitation;
	subsystems[HAND]   = initialExcitation;
	subsystems[EYE]	   = initialExcitation;		//mapping activity
	updateGlobalExcitation();

}

Excitation::~Excitation()
{
	// TODO Auto-generated destructor stub
}



	void Excitation::updateGlobalExcitation()
	{
		unsigned int elements = subsystems.size();
		globalExcitation = 0;
		for(auto& x: subsystems)
		{
			globalExcitation += x.second;
		}
		globalExcitation /= elements;
	}

	System Excitation::getMaxExcitation()
	{
		float max = 0;
		System element;
		for(auto& x: subsystems)
		{
			if(x.second>max)
				element = x.first;
		}
		return element;
	}

	/**
	 * map saturation should give a percentage of map saturation.  As the map
	 * becomes saturated, the novelty should decay.
	 */
	void Excitation::setEyeExcitation(int mapSaturation)
	{
		subsystems[EYE] = 1 - (mapSaturation/100.0);
		updateGlobalExcitation();
	}


	/**
	 * Based on the last saccade, how many steps did it take and how many fields were learnt
	 * This version is only used during learning.  Steps are currently ignored.
	 *
	 */
	void Excitation::updateEyeExcitation(int steps, int fieldsLearnt)
	{
		//if taking lots of steps, then poor control, but lots of learning, i.e.
		float excitation = log(steps/10.0)*-1;	//100 -> -1
		if(fieldsLearnt>0)
		{
			float fields = 1 - (1/fieldsLearnt);
			excitation += fields;
			excitation /= 2;
		}
		else
		{
			decay(EYE);
		}
		float current = subsystems[EYE];
		current += excitation;
		current /= 2;
		subsystems[EYE] = current;

		updateGlobalExcitation();
	}

	/**
	 * This version is used when not learning, and is purely based on the number of steps.
	 * excitation is high when not many steps are required.
	 */
	void Excitation::updateEyeExcitation(int steps)
	{
		//1 step is the best, more than that is bad.  1st saccade could be as bad as 100.
		float excitation = log(steps/10.0)*-1;	//100 -> -1
		float current = subsystems[EYE];
		excitation += current;
		excitation /= 2;
		subsystems[EYE] = excitation;

		updateGlobalExcitation();
	}

	/**
	 * fov is % of full view, larger number better.
	 * acuity is 0-200, smaller number is preferable.
	 */
	void Excitation::setFovealExcitation(int acuity, int fov)
	{
		float eFOV = fov/100;
		float eAcuity =  1 - (acuity/200);

		float excitation = (eFOV + eAcuity)/2;

		subsystems[FOVEAL] = excitation;

		updateGlobalExcitation();
	}

	/**
	 *  the change, hopefully improvement in the visual acuity and fov
	 */
	void Excitation::updateFovealExcitation(int acuity_change, int fov_change)
	{
		float eChange = 0;

		//larger fov is better.  fov = percentage.

		eChange += (fov_change/100);

		//smaller acuity is better, max acuity = 99? min=1?
		//	Acuity must be odd, so change will always be even
		eChange += (acuity_change/99);	//small improvements less exciting than large changes,
										// but still exciting.

		if(eChange>1)
			eChange = 1;

		float current = subsystems[FOVEAL];
		subsystems[FOVEAL] = (current + eChange)/2;

		updateGlobalExcitation();
	}


	void Excitation::setRetinaExcitation()
	{
		//TODO
		//more vision modes is more exciting.
		currentVisionFlags = params.m_VISION_FLAGS;

		float excitation = 0;
		if(currentVisionFlags & COLOUR)
			excitation+=0.1;
		if(currentVisionFlags & MOTION)
			excitation+=0.2;
		if(currentVisionFlags & EDGE)
			excitation+=0.2;
		if(currentVisionFlags & SHAPE)
			excitation+=0.2;

		subsystems[RETINA] = excitation;

		updateGlobalExcitation();

	}
	void Excitation::updateRetinaExcitation()
	{
		//TODO NOTE: if disabling a type of visual feature, this will also cause excessive excitation.
		//addition of vision modes is very exciting.
		//how many visual stimuli are currently present, and what type are they?

		float excitation = 0;

		//start by comparing current to params
		unsigned char change = currentVisionFlags ^ params.m_VISION_FLAGS;	//XOR
		//Only expecting one to change at a time, but will need to cap at 1.
		if(change & COLOUR)
			excitation+=0.4;
		if(change & MOTION)
			excitation+=0.4;
		if(change & EDGE)
			excitation+=0.4;
		if(change & SHAPE)
			excitation+=0.4;

		if(excitation>1)
			excitation = 1;

		float current = subsystems[RETINA];
		excitation += current;
		excitation /= 2;
		subsystems[RETINA] = excitation;

		updateGlobalExcitation();

		//TODO: Next ideally need to know how many of each type of stimulus is present.
		//in target selection, more advanced types should be preferred over the basic types.

	}



	/**
	 *
	 * stage [0,1,2] to indicate the three stages of reach development.
	 * 				This is used to cap the overall level of excitation.
	 * mapSaturation [0..1] indication of % of fields learn on map
	 * 				As the map becomes more saturation, the novelty decays.
	 */
	void Excitation::setReachExcitation(int stage, float mapSaturation)
	{
		float excitation = stage * 0.3 + 0.3;

		excitation *= (1-mapSaturation);
		subsystems[ARM] = excitation;

		//if(stage==0)
			subsystems[HAND] = excitation;

		//Distance will need to have a large impact on the hand and arm.
		updateGlobalExcitation();
	}


	/**
	 * As the reach system improves, the reach will be able to get closer to the target.
	 * This distance from target measurement is used to adjust novelty of hand
	 */
	void Excitation::updateReachExcitation(float distance)
	{
		//use the distance from the target to adjust the excitation of the hand and the arm.
		//if getting close to the target, the excitation should increase

		//distance is measured in metres.
		//smaller number highly preferable.  Target is: 0.009
		//really bad reach is +10cm away, i.e. 0.1

		float reachTarget = 0.009;
		distance -= reachTarget;
		distance *= 10;	//bring into range 0-1;

		float excitation = 1-distance;
		float current = subsystems[ARM];
		excitation += current;
		excitation /= 2;
		subsystems[ARM] = excitation;

		updateGlobalExcitation();
	}


 	void Excitation::decay(System subsystem)
	{
		float current = subsystems[subsystem];
		if(current > 0.025)
		{
			current -= 0.015;
		}
		updateGlobalExcitation();
	}

} /* namespace smc */





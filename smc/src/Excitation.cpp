/*
 * Excitation.cpp
 *
 *  Created on: 4 May 2015
 *      Author: icub
 */

#include "Excitation.h"


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
		if(elements == 0)
		{
			cout << "Fixing elements size" << endl;
			elements = 5;
		}
		globalExcitation = 0;
		for(map<System,float>::iterator it=subsystems.begin(); it!=subsystems.end(); it++)
		{
			globalExcitation += it->second;
		}
		globalExcitation /= elements;

		if(isinf(globalExcitation))
		{
			stimulateSystem();
		}
	}

	System Excitation::getMaxExcitation()
	{
		float max = 0.0;
		System element;
		for(map<System,float>::iterator it=subsystems.begin(); it!=subsystems.end(); it++)
		{
			if(it->second > max)
			{
				max = it->second;
				element = it->first;
			}
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
			if(excitation <0)
			excitation = 0.1;
		}
		else
		{
			decay(EYE);
		}
		float current = subsystems[EYE];
		if(current < 0.0001)
			current = 0.0001;
		current += excitation;
		current /= 2;
		subsystems[EYE] = current;

		updateGlobalExcitation();
	}

	/**
	 * This version is used when not learning, and is purely based on the number of steps.
	 * excitation is high when not many steps are required.
	 */
	void Excitation::updateEyeExcitation(int steps, bool success)
	{
		//1 step is the best, more than that is bad.  1st saccade could be as bad as 100.
		float excitation = log(steps/10.0)*-1;	//100 -> -1
		if(!success)
		{
			excitation = subsystems[EYE] /=2;
		}
		float current = subsystems[EYE];
		if(current < 0.0001)
			current = 0.0001;

		excitation += current;
		excitation /= 2;
		if(excitation <0)
			excitation = 0.01;
		else if(excitation >=1)
			excitation = 0.95;
		subsystems[EYE] = excitation;

		updateGlobalExcitation();
	}

	/**
	 * fov is % of full view, larger number better.
	 * acuity is 0-200, smaller number is preferable.
	 */
	void Excitation::setFovealExcitation(int acuity, int fov)
	{
		float eFOV = (float)fov/100.0/2.0;
		float eAcuity =  (1.0 - ((float)acuity/200.0))/2.0;

		float excitation = (eFOV + eAcuity)/2.0;

		if(excitation <=0)
			excitation = 0.1;

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
		if(eChange <0)
			eChange = 0.1;

		float current = subsystems[FOVEAL];
		if(current < 0.0001)
			current = 0.0001;
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
	void Excitation::updateRetinaExcitation(int visualTargets)
	{
		//TODO NOTE: if disabling a type of visual feature, this will also cause excessive excitation.
		//addition of vision modes is very exciting.
		//how many visual stimuli are currently present, and what type are they?

		float excitation = 0;

		excitation += ((float)visualTargets * 0.2);

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
		if(current < 0.0001)
			current = 0.0001;
		excitation += current;
		excitation /= 2;
		subsystems[RETINA] = excitation;

		updateGlobalExcitation();

	}

	void Excitation::updateRetinaExcitation(int colourTargets, int motionTargets)
	{



		float excitation = 0;
		excitation += ((float)colourTargets * 0.1);
		excitation += ((float)motionTargets * 0.2);

		if(excitation == 0)
		{
			decay(RETINA);
			decay(HAND);
		}
		else
		{
			float current = subsystems[RETINA];
			if(current < 0.0001)
				current = 0.0001;
			excitation += current;
			excitation /= 2;
			subsystems[RETINA] = excitation;
			stimulateSubSystem(HAND);
		}

		updateGlobalExcitation();

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
		float excitation = stage * 0.3;

		excitation += ((1-mapSaturation)/4);
		subsystems[ARM] = excitation;

		if(excitation <0)
			excitation = 0.1;

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
		if(current < 0.0001)
			current = 0.0001;

		excitation += current;
		excitation /= 2;

		if(excitation <0)
			excitation = 0.1;
		subsystems[ARM] = excitation;

		updateGlobalExcitation();
	}


	float Excitation::getExcitation(System system)
	{
		return subsystems[system];
	}

 	void Excitation::decay(System subsystem)
	{
		float current = subsystems[subsystem];
		if(current > 0.025)
		{
			current -= 0.015;
		}
		subsystems[subsystem] = current;
		updateGlobalExcitation();
	}
 	void Excitation::stimulateSystem()
 	{
 		for(map<System,float>::iterator it=subsystems.begin(); it!=subsystems.end(); it++)
		{
 			if(it->second <0.0)
 			{
 				it->second = 0.001;
 			}
 			else if(it->second < 0.1)
 			{
 				it->second*=3;
 			}
 			else if(it->second < 0.2)
 			{
 				it->second*=2;
 			}
		}
 		updateGlobalExcitation();
 	}

 	void Excitation::stimulateSubSystem(System system)
 	{
 		float current = subsystems[system];
 		if(current <0.0)
 			{
 				current = 0.001;
 			}
 			else if(current < 0.1)
 			{
 				current*=3;
 			}
 			else if(current < 0.2)
 			{
 				current*=2;
 			}
 		subsystems[system] = current;
 		updateGlobalExcitation();
 	}

 	void Excitation::printExcitations()
 	{
 		for(map<System,float>::iterator it=subsystems.begin(); it!=subsystems.end(); it++)
		{
			cout << "[" << it->first << ":" << it->second << "] ";
		}
 		cout << endl;
 	}


 	std::string Excitation::excitationToString(System sys)
	{
		switch(sys)
		{
			case RETINA:
				return "retina";
				break;
			case FOVEAL:
				return "foveal";
				break;
			case ARM:
				return "arm";
				break;
			case HAND:
				return "hand";
				break;
			case EYE:
				return "eye";
				break;
			default:
				return "unknown";
				break;
		}
		return "unknown";
	}



} /* namespace smc */





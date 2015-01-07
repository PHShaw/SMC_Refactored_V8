/*
 * reachField.h
 *
 * 	TODO: Note, angle for left arm should be negated to avoid left and right arm positions being confused,
 *  especially when it comes to linking in to gaze space
 *
 *
 * From the arm kinematics: x-up, y-forward, z-left (iCub convention)
 * Therefore, in the fields, x and z are used for the arm position coordinates, (z, x)
 *
 *  Created on: 2 Aug 2011
 *      Author: icub
 */

#ifndef REACHFIELD_H_
#define REACHFIELD_H_

#include <iostream>		//provides ostream from namespace std
#include <vector>

#include "CalcHand.h"
#include "Field.h"

struct ReachConfig{

	ReachConfig(){
		shoulderPitch = 0;
		shoulderRoll = 10;
		shoulderYaw = 0;
		elbow = 15;
		wristProSup = 0;
		wristPitch = 0;
		wristYaw = 0;
		handFinger = 0;
		thumbOppose = 0;
		thumbProximal = 0;
		thumbDistal = 0;
		indexProximal = 0;
		indexDistal = 0;
		middleProximal = 0;
		middleDistal = 0;
		pinky = 0;
		rightArm = true;

		double torso[3] = {0,0,0};
		double kinY;
		double* arm = new double[16];
		toArray(arm);
		calcHandPos(torso, arm,rightArm,&kinX,&kinY,&kinZ);
	}

	ReachConfig(const double* config, bool rArm){
		shoulderPitch 	= (int)(config[0]);
		shoulderRoll 	= (int)(config[1]);
		shoulderYaw 	= (int)(config[2]);
		elbow 			= (int)(config[3]);
		wristProSup 	= (int)(config[4]);
		wristPitch 		= (int)(config[5]);
		wristYaw 		= (int)(config[6]);
		handFinger 		= (int)(config[7]);
		thumbOppose		= (int)(config[8]);
		thumbProximal 	= (int)(config[9]);
		thumbDistal 	= (int)(config[10]);
		indexProximal 	= (int)(config[11]);
		indexDistal 	= (int)(config[12]);
		middleProximal 	= (int)(config[13]);
		middleDistal 	= (int)(config[14]);
		pinky 			= (int)(config[15]);
		rightArm = rArm;

		double torso[3] = {0,0,0};
		double kinY;
		double* arm = new double[16];
		toArray(arm);
		calcHandPos(torso, arm,rightArm,&kinX,&kinY,&kinZ);
	}
//	ReachConfig(double config0, double config1, double config2, double config3, double config4, double config5,
//						double config6, double config7, double config8, double config9, double config10,
//						double config11, double config12, double config13, double config14, double config15){
//		shoulderPitch 	= (int)(config0);
//		shoulderRoll 	= (int)(config1);
//		shoulderYaw 	= (int)(config2);
//		elbow 			= (int)(config3);
//		wristProSup 	= (int)(config4);
//		wristPitch 		= (int)(config5);
//		wristYaw 		= (int)(config6);
//		handFinger 		= (int)(config7);
//		thumbOppose		= (int)(config8);
//		thumbProximal 	= (int)(config9);
//		thumbDistal 	= (int)(config10);
//		indexProximal 	= (int)(config11);
//		indexDistal 	= (int)(config12);
//		middleProximal 	= (int)(config13);
//		middleDistal 	= (int)(config14);
//		pinky 			= (int)(config15);
//	}

	//X
	double rotation()const{
//		double rotation = shoulderPitch + shoulderYaw + wristProSup;
//		if(!rightArm)
//			rotation*=-1;
//		return rotation;
		return kinZ;
	}

	//Y
	double flexion()const{
		//return shoulderRoll + elbow + wristPitch;
		return kinX;
	}

	//array must already be initialised, i.e double config = new double[16];
	void toArray(double config[]){
		config[0] = shoulderPitch;
		config[1] = shoulderRoll;
		config[2] = shoulderYaw;
		config[3] = elbow;
		config[4] = wristProSup;
		config[5] = wristPitch;
		config[6] = wristYaw;
		config[7] = handFinger;
		config[8] = thumbOppose;
		config[9] = thumbProximal;
		config[10] = thumbDistal;
		config[11] = indexProximal;
		config[12] = indexDistal;
		config[13] = middleProximal;
		config[14] = middleDistal;
		config[15] = pinky;
	}

	double shoulderPitch, shoulderRoll, shoulderYaw, elbow, wristProSup, wristPitch, wristYaw,
			handFinger, thumbOppose, thumbProximal, thumbDistal, indexProximal, indexDistal,
			middleProximal, middleDistal, pinky;

	double kinX,kinZ;

	bool rightArm;
};

//non-member functions
std::ostream& operator <<(std::ostream& outs, const ReachConfig& reach);
bool operator ==(const ReachConfig& rc1, const ReachConfig& rc2);


class ReachField: public Field
{
public:
	ReachField();
	ReachField(size_t fieldID, double depth, double angle, double* reach, bool arm, double radius=0.5);	//Pseudo depth = flexion, pseudo angle = rotation
	virtual ~ReachField();

	void getReachDirection(double* depth, double* angle) const;
	ReachConfig* getPreferredReach(bool *rightArm);		//Returns a reach configuration for the preferred arm,
													//with the bool indicating if it is the right arm or not.
													//Preference currently based on minimising the position of
													//all the joints.

	bool getPreferredArm() const;					//based on the number of reach configurations for each arm

	void addReachConfiguration(double* reach, bool rightArm);

	ReachConfig* getLeftReach();
	ReachConfig* getRightReach();

	int getLeftReachCount() const;
	int getRightReachCount() const;

	int getUsage() const;
	void use();

	double getRadius() const;
    void setRadius(double radius);

	std::vector<ReachConfig*> getLeftReaches() const;
    std::vector<ReachConfig*> getRightReaches() const;

    ReachConfig* getReach(std::vector<ReachConfig*> *reaches, double *minCost);

    bool isIntersecting() const;
    void setIntersects(bool overlap);	//It may be possible to guess/calculate this if have a left and

    //right reach to the same depth and angle?

private:
    std::vector<ReachConfig*> leftReaches;	//array of arrays containing all joint positions
    std::vector<ReachConfig*> rightReaches;

    bool leftReached;
    bool rightReached;
    bool intersects;	// Indicates a reach point where the two hands overlap.  At some point this may be calculated

    int usage;
    double radius;

};

//non-member functions
std::ostream& operator <<(std::ostream& outs, const ReachField& gf);

//Arm joint IDs
//Rotational joints:
 const int shoulderPitch = 0;
 const int shoulderYaw = 2;
 const int wristProSup = 4; //wrist pronosupination
 const int wristYaw = 6;
//Flexion joints:
 const int shoulderRoll = 1;
 const int elbow = 3;
 const int wristPitch = 5;
//Hand joint IDs:
 const int handAdduction = 7; //adduction/abduction (spreading fingers out)
 const int thumbOppose = 8;
 const int thumbProximal = 9;
 const int thumbDistal = 10;
 const int indexProximal = 11;
 const int indexDistal = 12;
 const int midProximal = 13;
 const int midDistal = 14;
 const int pinky = 15;


#endif /* REACHFIELD_H_ */

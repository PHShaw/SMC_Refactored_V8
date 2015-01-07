/*
 * GazeMap.h
 *
 * An unconstrained map that is built up dynamically.  No fields are predefined, and the
 * location of the fields is not fixed.  Searching for nearby fields is performed based
 * on radius from location
 *
 * As the gaze map develops it becomes the 'body' map containing reach configurations that
 * are gradually linked to gaze configurations
 *
 *
 *
 * From the arm kinematics: x-up, y-forward, z-left (iCub convention)
 * Therefore, in the fields, x and z are used for the arm position coordinates, (z, x)
 *
 *  Created on: 30 Mar 2011
 *      Author: icub
 */


#ifndef GAZEMAP_H_
#define GAZEMAP_H_

#include <vector>

#include "GazeField.h"
#include "ReachField.h"



struct GazeReachLink{
	GazeReachLink(){
		gaze = new GazeField();
		reach = new ReachField();
//		gazeX = 0;
//		gazeY = 0;
//		flexion = 0;
//		angle = 0;
		gazeDepth = 0;
	}
	GazeReachLink(GazeField* gf, ReachField* rf, double depth)
	{
		gaze = gf;
		reach = rf;
//		gf->getGazeDirection(&gazeX, &gazeY);
//		rf->getReachDirection(&flexion, &angle);
		gazeDepth = depth;
	}
//	GazeReachLink(double gx, double gy, double d, double a, double depth)
//	{
//		gazeX = gx;
//		gazeY = gy;
//
//		flexion = d;
//		angle = a;
//
//		gazeDepth = depth;
//	}

	double getGazeX() const
	{
		double gazeX,gazeY;
		gaze->getGazeDirection(&gazeX,&gazeY);
		return gazeX;
	}
	double getGazeY() const
	{
		double gazeX,gazeY;
		gaze->getGazeDirection(&gazeX,&gazeY);
		return gazeY;
	}
	bool rightArm() const
	{
		return reach->getPreferredArm();
	}
	double getArmFlexion() const
	{
		return reach->getXcoord();
	}
	double getArmAngle() const
	{
		return reach->getYcoord();
	}
	double getGazeDepth() const
	{
		return gazeDepth;
	}


//	double gazeX, gazeY, flexion, angle;
//	bool rightArm;	// is this needed?
	//should reach and gaze fields be stored here as well?
	GazeField* gaze;
	ReachField* reach;
	double gazeDepth;
};

//non-member functions
bool operator ==(const GazeReachLink& l1, const GazeReachLink& l2);
std::ostream& operator <<(std::ostream& outs, const GazeReachLink& source);
bool compareGazeLinks(GazeReachLink* l1, GazeReachLink* l2);


class GazeMap
{
public:
	GazeMap();
	virtual ~GazeMap();

	void addGazeField(double* headMotorConfig);
	void addGazeField(double gazeX, double gazeY, double* headMotorConfig, double radius=3);
	void addGazeField(double gazeX, double gazeY, double worldX, double worldY, double worldZ, double vergence, double* headMotorConfig, double radius=3);
	void addReachField(double* armReachConfig, bool rightArm, double radius=0.5);
	void addGazeReach(double* headMotorConfig, double* armReachConfig, bool rightArm, double depth);

	GazeField*  getGazeField(double gazeX, double gazeY);
	GazeField*  getGazeField(double worldX, double worldY, double worldZ);
	GazeField*  getGazeField(double* headMotorConfig);
	ReachField* getReachField(double flexion, double angle);
	ReachField* getReachField(double* armReachConfig, bool rightArm);

	//get Nearest Field, allowing the radius to be an optional parameter
	bool getNearestGazeField(double gazeX, double gazeY, GazeField** field, double *distance,
								double radius=10);
	bool getNearestGazeField(double worldX, double worldY, double worldZ, GazeField** field, double *distance,
								double radius=10);
	bool getNearestReachField(double flexion, double angle, ReachField** field, double *distance, double radius=2);


	//get gaze field from arm configuration //how much tolerance is acceptable?
    bool getNearestGazeFieldForArmReach(double* armReach, bool isRightArm, GazeField** field,
											double* correlation, double radius=15);
    bool getNearestReachFieldForGaze(double* headMotorConfig, double depth, ReachField** field,
											double* correlation, double radius=15);
    bool getNearestReachFieldForGaze(double* headMotorConfig, ReachField** field,
											double* correlation, double radius=15);
    std::vector<GazeReachLink*> getReachSetForGaze(double* headMotorConfig, double radius=15);
    GazeReachLink* getLinkFromReach(ReachField* rf);

	bool addCoordination(double gazeX, double gazeY, double flexion, double angle, double depth);
	bool addCoordination(GazeField* gf, ReachField* rf, double depth);		//check to see if they are already linked

	bool updateLink(ReachField* rf, GazeField* newGF, double depth);


	bool getHandCoordination(GazeField* gf, ReachField** rf);
	bool getHandCoordination(GazeField* gf, ReachField** rf, double depth);	//returns reach field linked to gaze field
	bool getEyeCoordination(ReachField* rf, GazeField** gf);		//returns gaze field linked to reach field


    bool matchArmReach(ReachConfig* arm1, ReachConfig* arm2, double* comparison, double radius=2);

    std::vector<GazeField*> getGazeFields() const;
    std::vector<ReachField*> getReachFields() const;
    std::vector<GazeReachLink*> getLinks() const;
    unsigned int getNumGazeFields() const;
    unsigned int getNumReachFields() const;
    unsigned int getNumLinks() const;

    void gazeData(double* headMotorConfig, double* gazeX, double* gazeY);
    void armData(double* armReach, bool rightArm, double* flexion, double* angle);

private:
	std::vector<GazeField*> gazeFields;
	std::vector<ReachField*> reachFields;

	std::vector<GazeReachLink*> handEyeCoordination;
//	unsigned int numFields;




	size_t fieldIDCounter;
};

double headXtoEyeX(double headX);
	double headYtoEyeY(double headY);
	double eyeXtoHeadX(double eyeX);
	double eyeYtoHeadY(double eyeY);

#endif /* GAZEMAP_H_ */



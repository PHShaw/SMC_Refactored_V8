/*
 * EyeHeadController.h
 *
 *  Created on: 8 Jun 2011
 *      Author: icub
 */

#ifndef EYEHEADCONTROLLER_H_
#define EYEHEADCONTROLLER_H_

#include "eyeHeadCalc.h"
#include "eyeSaccading.h"
#include "headSaccading.h"



#include "GazeMap.h"
#include "FFM_IO.h"

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

const int eyeOnlyDuration = 30*60;		//time in seconds
const int headDuration = 30*60;		//Total experiment time, 1hr
const bool MOTOR_DYNAMIC_MAP = true;
const bool RETINA_DYNAMIC_MAP = false;
const int INPUT_MAP_TYPE = POLAR_MAP;
const int OUTPUT_MAP_TYPE = POLAR_MAP;	//Options are POLAR_MAP or DYNAMIC

struct altHeadChainLink{
	altHeadChainLink(double ex1, double ey1, double hx1, double hy1)
	{
		eyeX1 = ex1;
		eyeY1 = ey1;
		headX1 = hx1;
		headY1 = hy1;
	}
	void update(double ex2, double ey2, double hx2, double hy2)
	{
		eyeX2 = ex2;
		eyeY2 = ey2;
		headX2 = hx2;
		headY2 = hy2;
	}
	double eyeX1, eyeY1, eyeX2, eyeY2;
	double headX1, headY1, headX2, headY2;
};

class EyeHeadSaccading
{
public:
	EyeHeadSaccading(GazeMap* gm, Target* target);
	~EyeHeadSaccading();


	void init();
	bool initMaps();
	void initLogs();
	bool loadFile(std::string filename);

	bool saveMaps();


	int learnEyeSaccades(int maximum = 100);		//returns the number of saccades made
	int learnEyeHeadSaccades(int maximum = 100);

	bool learn_iStyleHeadLinks();
	void learnOuterHeadLinks();

	bool eyeSaccade();
	bool eyeHeadSaccade();
	bool AltEyeHeadSaccade();

//	bool simpleEyeOnlySaccade(string colour);
//	bool simpleEyeHeadSaccade(string colour);

	bool centreEyesInHead();
	bool centreEyesInHeadOnTarget(std::string colour, bool retry=false);


	bool checkInputFields(double targX, double targY, PolarField* eyeOutput, PolarField* headOutput);
	bool checkCombiLinks(double targX, double targY);
	bool checkCombiLinks(double targX, double targY, std::string colour, bool check=true);
	bool fixate(double targX, double targY, std::string colour, bool simple=true);
	bool fixate(int xl, int yl, int xr, int yr);
	bool fixate();	//Fixate on something
	bool fixate(std::string colour);	//fixate on a target of colour...

	bool fixateEyeHead(int xr, int yr);

	//Functions for using at gaze positions
	bool addGazeField();
	bool goToGazeField(GazeField* gf);
	bool goToGazeField(double gazeX, double gazeY);
	void goToHeadConfig(HeadConfig* hc);
	bool isGazeField();
	GazeField* getGazeField();
	void getGazeDirection(double* gazeX, double* gazeY);
	bool getGazeDirection(double targX, double targY, double* hmc);
	bool getGazeDirection(double targX, double targY, double* gazeX, double* gazeY);


	bool getDepth(std::string colour, double* depth);
	bool autoCenter(std::string colour);

	bool allStationary();
	void toRest();

	void babble();
	void smallBabble(double maxRange=5);

	bool verge(std::string colour, double* depth, bool centreHead=true);
	bool getVergence(double* vergence);

	//functions to get stats

	//functions to control constraints lifting and imposing


	headSaccading* headSacker() const {return headSac;}
	GazeMap* getGazeMap(){return gm;}
	ffm* getEyeMap(){return eye_ppm;}
	ffm* getHeadMap(){return head_ppm;}
	vor* getVor(){return v;}


	void eyeStats(bool links=false);
	void headStats();
	void closeLogs();


	//Test functions:
	void testMotorLinks();


	//botch functions:
	yarp::dev::PolyDriver* getMotorDriver(){return motordriver;}
	Target* getTarget(){return target;}
	eyeSaccading* getEyeSaccader(){return eyeSac;}
	eyeController* getEyeController(){return eyeCont;}
	headSaccading* getHeadSaccader(){return headSac;}
	headController* getHeadController(){return headCont;}

private:
	bool initYarp();

	Target* target;

	eyeController* eyeCont;
	headController* headCont;

	eyeSaccading* eyeSac;
	headSaccading* headSac;

	vor* v;

	//Variables defined in params_config.h
//	bool learn;
//	bool synchronous;
//	bool nearestNeighbour;
	bool randomMove;

	ffm* eye_ppm;
	ffm* head_ppm;

	GazeMap* gm;

	bool load;
	std::string path;
	std::string filename;


	yarp::dev::PolyDriver *motordriver;

	//eye threshold
	double eyeThreshold;		//Rolling average of the number of steps required for eye to fixate on target
	double rollingAverage;			//based on number of steps used for the eye to fixate on a target
	list<int> rollingBlock;			//The array used to calculate rolling averages for the eye threshold

	//head threshold
	double eyeHeadThreshold;
	int combiThreshold;


	//Stats
	int eyeOnlyCounter;
	int saccadeCounter;
	int combiSuccessful;
	int combiClose;
	int combiFailed;
	int eyeOnly;

	double eyeHeadSuccessRate;


	std::ofstream eyestatslog;
	std::ofstream headstatslog;
	std::ofstream eyeHeadlog;

	std::ofstream headLinkLog;

};

#endif /* EYEHEADCONTROLLER_H_ */




/*
 * gazeField.h
 *
 *  Created on: 30 Mar 2011
 *      Author: icub
 */

#ifndef GAZEFIELD_H_
#define GAZEFIELD_H_

#include <iostream>		//provides ostream from namespace std
#include <math.h>
#include <string>
#include <vector>

#include "Field.h"
#include "CalcGazeChange.h"

struct HeadConfig{

	HeadConfig(){
		neckPitch = 0;
		neckRoll = 0;
		neckYaw = 0;
		eyeTilt = 0;
		eyeVersion = 0;
		eyeVergence = 0;
	}

	HeadConfig(double* config){
		neckPitch 	= (float)((int)(config[0]*1000))/1000;
		neckRoll 	= (float)((int)(config[1]*1000))/1000;
		neckYaw 	= (float)((int)(config[2]*1000))/1000;
		eyeTilt 	= (float)((int)(config[3]*1000))/1000;
		eyeVersion 	= (float)((int)(config[4]*1000))/1000;
		eyeVergence = (float)((int)(config[5]*1000))/1000;
	}

	HeadConfig(double config0, double config1, double config2, double config3, double config4, double config5){
		neckPitch 	= (float)((int)(config0*1000))/1000;
		neckRoll 	= (float)((int)(config1*1000))/1000;
		neckYaw 	= (float)((int)(config2*1000))/1000;
		eyeTilt 	= (float)((int)(config3*1000))/1000;
		eyeVersion 	= (float)((int)(config4*1000))/1000;
		eyeVergence = (float)((int)(config5*1000))/1000;
	}



	double headX()const{return neckYaw;}
	double headY()const{return neckPitch;}
	double eyeX()const{return eyeVersion;}
	double eyeY()const{return eyeTilt;}

	//The array passed must already be initialised "= new double[6];" !!!
	void toArray(double config[]){
		config[0] = neckPitch;
		config[1] = neckRoll;
		config[2] = neckYaw;
		config[3] = eyeTilt;
		config[4] = eyeVersion;
		config[5] = eyeVergence;
	}

	double neckPitch, neckRoll, neckYaw, eyeTilt, eyeVersion, eyeVergence;
};

//non-member functions
std::ostream& operator <<(std::ostream& outs, const HeadConfig& head);
bool operator ==(const HeadConfig& h1, const HeadConfig& h2);





class GazeField: public Field
{
public:
	GazeField();
	GazeField(size_t fieldID, double gazeX, double gazeY, double* eyeHeadMotor, double rad=3);
	GazeField(size_t fieldID, double gazeX, double gazeY, double worldX, double worldY, double worldZ, double vergence, double* eyeHeadMotor, double rad=3);
	virtual ~GazeField();

	void getGazeDirection(double* gazeX, double* gazeY) const;


	void addGazeConfiguration(double* gaze);


	HeadConfig* getPreferredGaze();




	int getGazeCount() const;
	int getUsage() const;
	void use();

    std::string getComments() const;
    std::vector<HeadConfig*> getEyeHeadPositions() const;

    void setComments(std::string comments);
    double getRadius() const;
    void setRadius(double radius);


    friend bool operator ==(const GazeField& gf1, const GazeField& gf2);
    friend bool operator <(const GazeField& gf1, const GazeField& gf2);

    float wx,wy,wz;
    float vergence;

private:
    std::vector<HeadConfig*> eyeHeadPositions; //array of arrays containing all the joint positions for the eye and head.


    bool gazed;
//    int gazePosCount;

    std::string comments;

    int usage;

    double radius;


};


//non-member functions
std::ostream& operator <<(std::ostream& outs, const GazeField& gf);
//bool operator ==(const GazeField& gf1, const GazeField& gf2);
//bool operator <(const GazeField& gf1, const GazeField& gf2);

//Head Joint IDs
 const int headX = 2; //neck yaw
 const int headY = 0; //neck pitch
 const int headRoll = 1;
 const int eyeX = 4; //eyes version
 const int eyeY = 3; //eyes tilt
 const int eyeVerge = 5;



#endif /* GAZEFIELD_H_ */

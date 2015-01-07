/*
 * Gaze_IO.cpp
 *
 * there will be a file for the arm reaches and a file for the gaze configurations.
 * each possible motor configuration will be stored independently and linked to
 * the gaze direction.
 *
 *
 *  Created on: 4 Apr 2011
 *      Author: icub
 */

#include "Gaze_IO.h"

Gaze_IO::Gaze_IO()
{
	// TODO Auto-generated constructor stub

}

Gaze_IO::~Gaze_IO()
{
	// TODO Auto-generated destructor stub
}


bool Gaze_IO::saveMappingToXML(GazeMap *gm, string filename){

	//Gaze configurations
	CXmlMap *cxm_head = new CXmlMap();
	cxm_head->init(3,3,3,gm->getNumGazeFields());


	float *headIn = new float[3];	//neck motors
	float *headOut = new float[3];	//eye motors
	float *headAtts = new float[3];		//gaze direction + radius


	vector<GazeField*> gazeFields = gm->getGazeFields();

	for(size_t i=0; i<gazeFields.size(); i++)
	{
		GazeField* temp = gazeFields.at(i);
		double x, y, rad;
		temp->getGazeDirection(&x, &y);
		rad = temp->getRadius();

		//Gaze configurations
		vector<HeadConfig*> gazes = temp->getEyeHeadPositions();
		for(size_t j=0; j<gazes.size(); j++)
		{
			double* config = new double[6];
			HeadConfig* hc = gazes.at(j);
			hc->toArray(config);

			headIn[0] = (float)config[0];
			headIn[1] = (float)config[1];
			headIn[2] = (float)config[2];

			headOut[0]= (float)config[3];
			headOut[1]= (float)config[4];
			headOut[2]= (float)config[5];

			headAtts[0] = (float)x;
			headAtts[1] = (float)y;
			headAtts[2] = (float)rad;
//			headAtts[3] = temp->wx;
//			headAtts[4] = temp->wy;
//			headAtts[5] = temp->wz;
//			headAtts[6] = temp->vergence;

			cxm_head->addLink(headIn,headOut,headAtts);
		}
	}

	//Gaze configurations
	string gazeFile = filename + "_gaze.xml";

	char * cFilename;
	cFilename = new char [gazeFile.size()+1];
	strcpy (cFilename, gazeFile.c_str());

	cout << "Attempting to write gaze fields to file " << cFilename << endl;
	cxm_head->writeXmlFile(cFilename);


	cout << "There are " << gm->getNumReachFields() << " reach fields to save" << endl;


	//Reach configurations
	CXmlMap *cxm_reach = new CXmlMap();
	cxm_reach->init(8,8,4,gm->getNumReachFields());	//attributes also indicate which arm 0=left, 1=right

	float *armIn = new float[8];	//arm motors (mostly)
	float *armOut = new float[8];	//hand motors
	float *armAtts = new float[4];		//gaze direction and indication of which arm and radius


	vector<ReachField*> reachFields = gm->getReachFields();
	for(size_t i=0; i<reachFields.size(); i++)
	{
		double x,y,rad;
		ReachField* temp = reachFields.at(i);
		temp->getReachDirection(&x, &y);
		rad = temp->getRadius();

		//right arm Reach configurations
		float arm = 1.0;		//temp->getPreferredArm();
		vector<ReachConfig*> reaches = temp->getRightReaches();
		for(size_t j=0; j<reaches.size(); j++)
		{
			double* config = new double[16];
			ReachConfig* rc = reaches.at(j);
			rc->toArray(config);
			armIn[0] = (float)config[0];
			armIn[1] = (float)config[1];
			armIn[2] = (float)config[2];
			armIn[3] = (float)config[3];
			armIn[4] = (float)config[4];
			armIn[5] = (float)config[5];
			armIn[6] = (float)config[6];
			armIn[7] = (float)config[7];

			armOut[0]= (float)config[8];
			armOut[1]= (float)config[9];
			armOut[2]= (float)config[10];
			armOut[3]= (float)config[11];
			armOut[4]= (float)config[12];
			armOut[5]= (float)config[13];
			armOut[6]= (float)config[14];
			armOut[7]= (float)config[15];

			armAtts[0] = (float)x;
			armAtts[1] = (float)y;
			armAtts[2] = arm;
			armAtts[3] = (float)rad;

			cxm_reach->addLink(armIn,armOut,armAtts);
		}

		//left arm Reach configurations
		arm = 0.0;
		reaches = temp->getLeftReaches();
		for(size_t j=0; j<reaches.size(); j++)
		{
			double* config = new double[16];
			ReachConfig* rc = reaches.at(j);
			rc->toArray(config);
			armIn[0] = (float)config[0];
			armIn[1] = (float)config[1];
			armIn[2] = (float)config[2];
			armIn[3] = (float)config[3];
			armIn[4] = (float)config[4];
			armIn[5] = (float)config[5];
			armIn[6] = (float)config[6];
			armIn[7] = (float)config[7];

			armOut[0]= (float)config[8];
			armOut[1]= (float)config[9];
			armOut[2]= (float)config[10];
			armOut[3]= (float)config[11];
			armOut[4]= (float)config[12];
			armOut[5]= (float)config[13];
			armOut[6]= (float)config[14];
			armOut[7]= (float)config[15];

			armAtts[0] = (float)x;
			armAtts[1] = (float)y;
			armAtts[2] = arm;
			armAtts[3] = (float)rad;

			cxm_reach->addLink(armIn,armOut,armAtts);
		}
	}

	//Reach configurations
	string reachFile = filename + "_reach.xml";
	cFilename = new char [reachFile.size()+1];
	strcpy (cFilename, reachFile.c_str());

	cxm_reach->writeXmlFile(cFilename);



	//hand eye links
	CXmlMap *cxm_links = new CXmlMap();
	cxm_links->init(2,2,1,gm->getNumLinks());

	float *gazeLink = new float[2];
	float *reachLink = new float[2];
	float *linkAtt = new float[1];

	vector<GazeReachLink*> links = gm->getLinks();
	for(size_t i=0; i<links.size(); i++)
	{
		GazeReachLink* l = links.at(i);
		gazeLink[0] = l->getGazeX();
		gazeLink[1] = l->getGazeY();

		reachLink[0] = l->getArmFlexion();
		reachLink[1] = l->getArmAngle();

		linkAtt[0] = l->gazeDepth;		//Could be used to indicate which arm it is,
							// but hoping the angle will be sufficient.

		cxm_links->addLink(gazeLink,reachLink,linkAtt);
	}

	//hand eye coordination
	string linkFile = filename + "_link.xml";
	cFilename = new char [linkFile.size()+1];
	strcpy (cFilename, linkFile.c_str());

	cxm_links->writeXmlFile(cFilename);
}



GazeMap* Gaze_IO::loadMappingFromXML(string filename)
{


	GazeMap* gm = new GazeMap();

//Try loading with extra world coord data, if that fails, try loading with basic data

	//Gaze configurations
	CXmlMap *cxm_head = new CXmlMap();
	cxm_head->init(3,3,7,10);

	string gazeFile = filename + "_gaze.xml";

	char * cFilename;
	cFilename = new char [gazeFile.size()+1];
	strcpy (cFilename, gazeFile.c_str());

//	try{
//		cxm_head->readXmlFile(cFilename);
//
//		double* headMotor = new double[6];
//		float* link = new float[13];
//		for(int i=0; i<cxm_head->nmbOfLinks();i++)
//		{
//			link = cxm_head->ptrLinkByIndex(i);
//
//			headMotor[0] = link[0];
//			headMotor[1] = link[1];
//			headMotor[2] = link[2];
//			headMotor[3] = link[3];
//			headMotor[4] = link[4];
//			headMotor[5] = link[5];
//
//			double gazeX = link[6];
//			double gazeY = link[7];
//			double radius = link[8];
//			float wx = link[9];
//			float wy = link[10];
//			float wz = link[11];
//			float vergence = link[12];
//
//			gm->addGazeField(gazeX, gazeY, wx, wy, wz, vergence, headMotor,radius);
//		}
//	}
//	catch(exception e)
//	{
//		cout << "Failed to load gaze map with extra world coords" << endl;
		cxm_head = new CXmlMap();
		cxm_head->init(3,3,3,10);

		try{
		cxm_head->readXmlFile(cFilename);
		}
		catch(IMapException ime)
		{
			cout << "Failed to load gaze map" << endl;
			return gm;
		}

		double* headMotor = new double[6];
		float* link = new float[9];
		for(int i=0; i<cxm_head->nmbOfLinks();i++)
		{
			link = cxm_head->ptrLinkByIndex(i);

			headMotor[0] = link[0];
			headMotor[1] = link[1];
			headMotor[2] = link[2];
			headMotor[3] = link[3];
			headMotor[4] = link[4];
			headMotor[5] = link[5];

			double gazeX = link[6];
			double gazeY = link[7];
			double radius = link[8];

			gm->addGazeField(gazeX, gazeY, headMotor,radius);
		}
//	}


	//Reach configurations
	CXmlMap *cxm_reach = new CXmlMap();
	cxm_reach->init(8,8,4,10);	//attributes also indicate which arm 0=left, 1=right

	//Reach configurations
	string reachFile = filename + "_reach.xml";
	cFilename = new char [reachFile.size()+1];
	strcpy (cFilename, reachFile.c_str());

	try{
		cxm_reach->readXmlFile(cFilename);
	}
	catch(IMapException ime)
	{
		cout << "failed to load reach file" << endl;
		return gm;
	}

	double* reachMotor = new double[16];
//	float*
	link = new float[20];
	for(int i=0; i<cxm_reach->nmbOfLinks();i++)
	{
		link = cxm_reach->ptrLinkByIndex(i);

		for(int j=0; j<16; j++)
			reachMotor[j] = link[j];


		double depth = link[16];
		double angle = link[17];

		bool arm = (bool) link[18];
		double radius = link[19];
		gm->addReachField(reachMotor, arm, radius);

	}


	CXmlMap *cxm_links = new CXmlMap();
	cxm_links->init(2,2,1,10);

	//link configurations
	string linkFile = filename + "_link.xml";
	cFilename = new char [linkFile.size()+1];
	strcpy (cFilename, linkFile.c_str());

	try{
		cxm_links->readXmlFile(cFilename);
	}
	catch(IMapException ime)
	{
		cout << "failed to load links file" << endl;
		return gm;
	}
	link = new float[5];
	for(int i=0; i<cxm_links->nmbOfLinks();i++)
	{
		link = cxm_links->ptrLinkByIndex(i);

		double gazeX, gazeY, reachX, reachY, depth;
		gazeX = link[0];
		gazeY = link[1];
		reachX = link[2];
		reachY = link[3];
		depth = link[4];
		gm->addCoordination(gazeX, gazeY, reachX, reachY, depth);
	}

	return gm;

}

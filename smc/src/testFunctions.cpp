/*
 * test.cpp
 *
 *  Created on: 23 Jun 2012
 *      Author: icub
 */

#include "handEyeCoordination.h"
#include "torsoController.h"
#include "tracker.h"



void testGazeFields(EyeHeadSaccading* ehCont)
{
	GazeMap* gm = ehCont->getGazeMap();
	vector<GazeField*> gazeFields = gm->getGazeFields();
	printf("There are %i gazeFields to work through, zzz\n", gazeFields.size());
	for(size_t i=0; i<gazeFields.size(); i++)
	{
		printf("Gaze field %i\n",i);
		GazeField* temp = gazeFields.at(i);
		vector<HeadConfig*> hcs = temp->getEyeHeadPositions();
		if(hcs.size()==1)
		{
			printf("There is only 1 head configuration here\n");
			continue;
		}
		printf("There are %i different headConfigs\n",hcs.size());
		for(size_t j=0; j<hcs.size(); j++)
		{
			printf("Going to config %i\n", j);
			HeadConfig* hc = hcs.at(j);
			ehCont->goToHeadConfig(hc);
			yarp::os::Time::delay(0.5);
		}

		yarp::os::Time::delay(3);
	}
}




void testHandEyeCoordination(handEyeCoordination* heCoor)
{
	GazeMap* gm = heCoor->getGazeMap();
	armReaching* ar = heCoor->getArmReaching();
	EyeHeadSaccading* ehCont = heCoor->getEyeHeadController();

	vector<GazeReachLink*> links = gm->getLinks();
	printf("There are %i links\n", links.size());
	for(size_t i=0; i<links.size(); i++)
	{
		GazeReachLink* l = links.at(i);
		cout << *l << endl;
		GazeField* g = l->gaze;
		ReachField* r = l->reach;


		if(g->getXcoord()==0 && g->getYcoord()==0)
		{
			printf("Null gaze field\n");
		}
		if(r->getXcoord()==0 && r->getYcoord()==0)
		{
			printf("Null reach field\n");
			continue;
		}

		printf("%i: There are %i gazeConfigs and %i armConfigs\n", i,
				g->getGazeCount(), (r->getRightReachCount() + r->getLeftReachCount()));
		ar->goToReachField(r);
		ehCont->goToGazeField(g);
		while(!heCoor->stationary())
			yarp::os::Time::delay(0.2);
		yarp::os::Time::delay(0.5);
		while(!heCoor->stationary())
			yarp::os::Time::delay(0.2);

		yarp::os::Time::delay(3);
	}

}



void testSpecificHandEyeCoordination(handEyeCoordination* heCoor)
{
	GazeMap* gm = heCoor->getGazeMap();
	armReaching* ar = heCoor->getArmReaching();
	EyeHeadSaccading* ehCont = heCoor->getEyeHeadController();

	vector<GazeReachLink*> links = gm->getLinks();
	printf("There are %i links\n", links.size());
	int num;
	char c;
	cout << "Enter a number between 0 and " << links.size() << " to view that link." << endl;
	cin >> num;
	while(num>=0 && num<links.size()){
		GazeReachLink* l = links.at(num);
		cout << *l << endl;
		GazeField* g = l->gaze;
		ReachField* r = l->reach;

		if(g->getXcoord()==0 && g->getYcoord()==0)
		{
			printf("Null gaze field\n");
		}
		if(r->getXcoord()==0 && r->getYcoord()==0)
		{
			printf("Null reach field\n");
		}
		else
		{
			printf("There are %i gazeConfigs and %i armConfigs\n",
				g->getGazeCount(), (r->getRightReachCount() + r->getLeftReachCount()));
			ar->goToReachField(r);
			ehCont->goToGazeField(g);
			while(!heCoor->stationary())
				yarp::os::Time::delay(0.2);
			yarp::os::Time::delay(0.5);
			while(!heCoor->stationary())
				yarp::os::Time::delay(0.2);


			printf("Would you like to test the alternative gaze positions? (y/n)\n");
			cin >> c;
			if(c=='y')
			{
				vector<HeadConfig*> gazes = g->getEyeHeadPositions();
				for(int i=0; i<gazes.size(); i++)
				{
					printf("Going to position %i\n", i);
					HeadConfig* hc = gazes.at(i);
					cout << *hc << endl;
					ehCont->goToHeadConfig(hc);
					while(!ehCont->allStationary())
						yarp::os::Time::delay(0.2);
					yarp::os::Time::delay(1);
				}

				printf("Returning to preferred gaze position\n");
				ehCont->goToGazeField(g);
				while(!ehCont->allStationary())
					yarp::os::Time::delay(0.2);
				yarp::os::Time::delay(1);
			}

			printf("Would you like to test the alternative reach positions? (y/n)\n");
			cin >> c;
			if(c=='y')
			{

				vector<ReachConfig*> reaches = r->getRightReaches();
				for(int i=0; i<reaches.size(); i++)
				{
					printf("Going to position %i\n", i);
					ReachConfig* rc = reaches.at(i);
					cout << *rc << endl;
					ar->goToReachConfig(rc);
					while(!heCoor->stationary())
						yarp::os::Time::delay(0.2);
					yarp::os::Time::delay(0.5);
					while(!heCoor->stationary())
						yarp::os::Time::delay(0.2);
					yarp::os::Time::delay(1);
				}
			}

		}

		yarp::os::Time::delay(3);

		cout << "Enter a number between 0 and " << links.size() <<
				" to view that link, or a number outside that range to finish." << endl;
		cin >> num;
	}

}

//TODO: need to incorporate depth calculation into this.
void refineCoordination(handEyeCoordination* heCoor)
{
	GazeMap* gm = heCoor->getGazeMap();
	armReaching* ar = heCoor->getArmReaching();
	EyeHeadSaccading* ehCont = heCoor->getEyeHeadController();

	vector<GazeReachLink*> links = gm->getLinks();
	sort(links.begin(), links.end(), compareGazeLinks);

	printf("There are %i links\n", links.size());
	char c;
	for(size_t i=0; i<links.size(); i++)
	{
		GazeReachLink* l = links.at(i);
		cout << *l << endl;
		GazeField* g = l->gaze;
		ReachField* r = l->reach;


		if(g->getXcoord()==0 && g->getYcoord()==0)
		{
			printf("Null gaze field\n");
		}
		if(r->getXcoord()==0 && r->getYcoord()==0)
		{
			printf("Null reach field\n");
			continue;
		}

		printf("%i: There are %i gazeConfigs and %i armConfigs\n", i,
				g->getGazeCount(), (r->getRightReachCount() + r->getLeftReachCount()));
		ar->goToReachField(r);
		ehCont->goToGazeField(g);
		while(!heCoor->stationary())
			yarp::os::Time::delay(0.2);
		yarp::os::Time::delay(0.5);
		while(!heCoor->stationary())
			yarp::os::Time::delay(0.2);

		printf("Current depth estimate is: %.2f\n",l->gazeDepth);

		yarp::os::Time::delay(0.2);
		double depth;
		ehCont->getDepth("yellow", &depth);
		printf("Now estimating depth at: %.2f\n", depth);

		cout << "Is this link acceptable?" << endl;
		cin >> c;
		if(c=='y')
			continue;


		ehCont->autoCenter("yellow");
		ehCont->centreEyesInHead();
		bool success = ehCont->autoCenter("yellow");
		yarp::os::Time::delay(0.2);
		ehCont->getDepth("yellow", &depth);
		printf("Now estimating depth at: %.2f\n", depth);

		if(success)
		{
			GazeField* ng = ehCont->getGazeField();
			cout << "Now obtained: " << *ng << endl;
			cout<<"Is this gaze field any better?" << endl;
			cin >> c;
			if(c=='y')
			{
				heCoor->updateLink(r, ng, depth);
			}
		}
		else
		{
			GazeField* ng = ehCont->getGazeField();
			cout << "Now obtained: " << *ng << endl;
			cout << "I don't think I'm fixated on the hand, but I've got this gaze field, is it any better?" << endl;
			cin >> c;
			if(c=='y')
			{
				heCoor->updateLink(r, ng, depth);
			}
		}

		yarp::os::Time::delay(3);
	}

	heCoor->saveGazeReachMap();
}





void addReachPositions(bool rightArm, handEyeCoordination* heCoor)
{
	armReaching* armReach = heCoor->getArmReaching();
//	Recording some new reach positions
//	armReach->getArmController()->babble(rightArm);

	ofstream reaches;
	reaches.open("reachesPointing.txt",  ios::out | ios::app);
	char c = 'y';
	double* armPos = new double[16];
	while(c!='x')
	{
		cout << "Move the arm (with eyes close to fixated) to a new position and press y or x to exit" << endl;
		cin >> c;
		if(c=='x')
			break;

		armReach->getArmController()->getCurrentPosition(armPos,rightArm);
		for(int i=0; i<16; i++)
		{
			reaches << armPos[i] << " ";
		}
		reaches << endl;

		//learn gaze link to arm position
		EyeHeadSaccading* ehCont = heCoor->getEyeHeadController();
		ehCont->fixate("yellow");
//		ehCont->centreEyesInHead();

//		yarp::os::Time::delay(0.2);
//		ehCont->autoCenter("yellow");

		GazeField* gf = ehCont->getGazeField();
		armReach->addReachField(rightArm);
		ReachField* rf = armReach->getReachField(rightArm);
		double depth;
		yarp::os::Time::delay(0.2);
		ehCont->verge("yellow", &depth);

		cout << "adding a link between " << *gf << " and " << *rf << endl;
		heCoor->addLink(gf, rf, depth);
	}

}


int colourToId(const char *colour) {
	if (strcmp(colour, "red") == 0)		return 0;
	if (strcmp(colour, "yellow") == 0)	return 1;
	if (strcmp(colour, "green") == 0)	return 2;
}
string idToColour(const int id) {
	if (id==0) return "red";
	if (id==1) return "yellow";
	if (id==2) return "green";
}
void testPeripheralPerceptions(EyeHeadSaccading* ehCont)
{

	//add random eye-head movements in?
	yarp::os::Network yarp;
	yarp::os::BufferedPort<yarp::os::Bottle> gazed;


	gazed.open("/gaze/live/out");
	yarp.connect("/gaze/live/out", "/gaze/live/in");

	GazeField* objects[3];
	objects[0]=NULL;
	objects[1]=NULL;
	objects[2]=NULL;
	Target* target = ehCont->getTarget();
	GazeMap* gm = ehCont->getGazeMap();


	while(true)
	{

		yarp::os::Bottle* bottle = target->getAllTargets();

		int size = bottle->size();
		if(size>0)
		{
			//TODO THIS NEEDS TO BE UPDATED TO HANDLE NEW VISION
			int colourElements =4;
			int numObjs = size/colourElements;
			string colour;
			double targX, targY, gX, gY;
			int size;

			for(int i=0; i<numObjs; i++)
			{
				colour = bottle->get(i*colourElements).asString();
				targX  = bottle->get(i*colourElements+1).asDouble();
				targY  = bottle->get(i*colourElements+2).asDouble();
				size   = bottle->get(i*colourElements+3).asInt();

				if(colour == "")
					continue;

				//get gaze direction to object
				double* headMotorConf2 = new double[6];
				bool success = ehCont->getGazeDirection(targX, targY, headMotorConf2);

				if(!success)
				{
					//may not have got current head position,
					// or may not have found a close enough link to estimate gaze direction from
					printf("Can see a %s target, but can't estimate a headMotorConf to it\n", colour.c_str());
					continue;
				}
				GazeField* gf = gm->getGazeField(headMotorConf2);
				double x,y;
				gf->getGazeDirection(&x, &y);
				if(gf->getXcoord()==0 && gf->getYcoord()==0)
				{
//					printf("Headmotorconf2: ");
//					for(int i=0; i<6; i++)
//					{
//						printf("%.2f ", headMotorConf2[i]);
//					}
//					printf("\n");
					gm->addGazeField(headMotorConf2);
					gf = gm->getGazeField(headMotorConf2);

					gf->getGazeDirection(&x, &y);
					if(gf->getXcoord()==0 && gf->getYcoord()==0)
					{
						printf("Unable to get gaze direction for %s target\n", colour.c_str());
						continue;
					}
				}


				int id = colourToId(colour.c_str());
				if(objects[id]==NULL)
				{
					objects[id] = gf;
					yarp::os::Bottle& gaze = gazed.prepare();
					gaze.clear();

					gaze.addInt(x);
					gaze.addInt(y);
					gaze.addInt(gf->getRadius());
					gaze.addString(colour.c_str());

					gazed.write();

				}
				else
				{
					//Seen this object before, now to see if its moved
					if(gf == objects[id])
					{
						printf("Can still see the %s target, and it hasn't moved an inch!\n", colour.c_str());
					}
					else
					{
						yarp::os::Bottle& gaze = gazed.prepare();
						gaze.clear();

						gaze.addInt(x);
						gaze.addInt(y);
						gaze.addInt(gf->getRadius());
						gaze.addString(colour.c_str());

						gazed.write();

						//gaze fields not exactly the same, but may be close enough to consider as the same
						double x2,y2;
						objects[id]->getGazeDirection(&x2,&y2);
						double dist = sqrt(pow(x-x2,2) + pow(y-y2,2));
						if(dist<5)
						{
							printf("New field is at a distance of %.2f for %s target, which is acceptably close to say it probably hasn't moved\n", dist, colour.c_str());
						}
						else
						{
							printf("New field is at a distance of %.2f for %s target, so think it may have moved\n", dist, colour.c_str());

						}
						objects[id] = gf;
					}
				}
			}
		}
		else
		{
			printf("No objects visible");
		}


		yarp::os::Time::delay(1.5);
	}
}



void testVergence(Target* target, EyeHeadSaccading* ehCont)
{
	double lTargX, lTargY;
	double rTargX, rTargY;

	eyeController* eye = ehCont->getEyeController();

	string colour="red";
	for(int i=0; i<30; i++)
	{
		target->getTarget(&rTargX, &rTargY, colour);
		target->getLeft(&lTargX, &lTargY, colour);
		cout << "Target difference: r-l="<<(rTargX - lTargY) << endl;

		double current;
		do
		{
			eye->getVergence(&current);
			eye->verg(current+1,true);
			yarp::os::Time::delay(0.1);

			target->getTarget(&rTargX, &rTargY, colour);
			target->getLeft(&lTargX, &lTargY, colour);

			cout << "Target difference: r-l="<<(rTargX - lTargY) << " vergence: " << (current+1) << endl;

		}while((rTargX - lTargY)<0.5);

		yarp::os::Time::delay(1.0);
	}


}



void testVergence2(Target* target, EyeHeadSaccading* ehCont)
{
	double lTargX, lTargY;
	double rTargX, rTargY;

	eyeController* eye = ehCont->getEyeController();

	string colour="red";
	double depth;
	for(int i=0; i<10; i++)
	{
		ehCont->getEyeController()->verg(0, true);
		ehCont->eyeSaccade();
		colour = target->getColour();
		ehCont->verge(colour, &depth);
		yarp::os::Time::delay(1.0);
	}


}




void testMatToTorso(EyeHeadSaccading* ehCont, torsoController* tor)
{
	//connect to the matlab ports
	yarp::os::Network yarp;
	yarp::os::BufferedPort<yarp::os::Bottle> portOut;
	yarp::os::BufferedPort<yarp::os::Bottle> portIn;
	yarp::os::Bottle* input;

	ofstream torsologfile;
	ofstream headlogfile;
	ofstream logfile;

	portOut.open("/torso/write");
	portIn.open("/torso/read");
	bool ok = false;
	yarp.connect("/torso/write", "/matlab/read");
	yarp.connect("/matlab/write", "/torso/read");

	//send confirmation of connections to matlab
	yarp::os::Bottle& b1 = portOut.prepare();
	b1.clear();
	b1.addDouble(1.0);
	portOut.write();

	ehCont->getEyeController()->verg(30,0);

	string path = "../data/";
	string fullpath = path + "torso.postorso";
	torsologfile.open(fullpath.c_str());

	fullpath = path + "head.poshead";
	headlogfile.open(fullpath.c_str());

	fullpath = path + "torsoMatlablog.txt";
	logfile.open(fullpath.c_str());


	logfile << "gazex gazey vergence torsoRot torsoTilt newVerg newTorRot newTorTilt"<< endl;

	for(int i=0; i<10; i++)
	{
		printf("Moving to new starting position\n");
		ehCont->babble();
		yarp::os::Time::delay(2.0);


		//get current gaze direction + vergence
		double gazeX, gazeY, vergence;
		ehCont->getEyeController()->getVergence(&vergence);
		ehCont->getGazeDirection(&gazeX, &gazeY);
		logfile << gazeX << " " << gazeY << " " << vergence << " ";
		double* hmp = new double[6];
		ehCont->getHeadController()->getCurrentPosition(hmp);

		headlogfile << "[POSITION" << (i*2) << "]" << endl;
		headlogfile << "jointPositions " << hmp[0] << " " << hmp[1] << " " << hmp[2] << " " << hmp[3] << " " << hmp[4] << " " << hmp[5] << endl;
		headlogfile << "jointVelocities jointVelocities 10.00 10.00 10.00 10.00 10.00 10.00\ntiming 0.50" << endl;


		printf("Current gaze direction: %.2f, %.2f\n", gazeX, gazeY);

		//get current torso rotation and tilt
		double torsoRotation, torsoTilt;
		tor->getCurrentPosition(&torsoRotation, &torsoTilt);
		logfile << torsoRotation << " " << torsoTilt;

		torsologfile << "[POSITION" << (i*2) << "]" << endl;
		torsologfile << "jointPositions " << torsoRotation << " 0.00 " << torsoTilt << endl;
		torsologfile << "jointVelocities jointVelocities 10.00 10.00 10.00\ntiming 0.50" << endl;


		//send positions to matlab
		yarp::os::Bottle& b = portOut.prepare();
		b.clear();
		b.addDouble(gazeX);
		b.addDouble(gazeY);
		b.addDouble(vergence);
		b.addDouble(torsoRotation);
		b.addDouble(torsoTilt);
		portOut.write();

		//receive relative movement from matlab
		printf("Waiting for answer from matlab\n");
		input = portIn.read(true);
		printf("Got answer from matlab: ");
		//returns relative torso movements and the absolute vergence angle to
		//fixate on the target with eye_tilt=0 and eye_version=0.

		vergence = input->get(0).asDouble();
		torsoRotation +=  input->get(1).asDouble();
		torsoTilt += input->get(2).asDouble();
		printf("%.2f, %.2f, %.2f\n", vergence, torsoRotation, torsoTilt);

		logfile << vergence << " " << torsoRotation << " " << torsoTilt << " " << endl;

		printf("Attempting to adjust with torso\n");
		tor->move(torsoRotation, torsoTilt, false);
		HeadConfig* hc = new HeadConfig(0,0,0,0,0,vergence);
		ehCont->goToHeadConfig(hc);

		yarp::os::Time::delay(3.0);

		ehCont->getHeadController()->getCurrentPosition(hmp);
		headlogfile << "[POSITION" << (i*2+1) << "]" << endl;
		headlogfile << "jointPositions " << hmp[0] << " " << hmp[1] << " " << hmp[2] << " " << hmp[3] << " " << hmp[4] << " " << hmp[5] << endl;
		headlogfile << "jointVelocities jointVelocities 10.00 10.00 10.00 10.00 10.00 10.00\ntiming 0.50" << endl;

		tor->getCurrentPosition(&torsoRotation, &torsoTilt);
		torsologfile << "[POSITION" << (i*2+1) << "]" << endl;
		torsologfile << "jointPositions " << torsoRotation << " 0.00 " << torsoTilt << endl;
		torsologfile << "jointVelocities jointVelocities 10.00 10.00 10.00\ntiming 0.50" << endl;
	}

	headlogfile << "[DIMENSIONS]\nnumberOfPoses 10\nnumberOfJoints 6" << endl;
	torsologfile << "[DIMENSIONS]\nnumberOfPoses 10\nnumberOfJoints 3" << endl;

	headlogfile.close();
	torsologfile.close();
	logfile.close();
}



void testTargetCoords(EyeHeadSaccading* ehCont)
{
	double torsoMotorConfig[3] = {0,0,0};
	double* headMotorConfig = new double[6];
	double x, y, z;

	double depth;	//vergence value

	bool success = ehCont->fixate();
	string colour = ehCont->getTarget()->getColour();

	if(success)
	{
		ehCont->verge(colour, &depth);
		ehCont->getHeadController()->getCurrentPosition(headMotorConfig);
		CalculateTargetWorldRef(torsoMotorConfig, headMotorConfig, &x, &y, &z);
		cout << "The target position in world space is: (" << x << ", " << y << ", " << z << ")" << endl;
	}



	cout << "Quick test of babble generator:" << endl;
	double* randomBabble = new double [15];
	randomBabble = babbleGenerator(15, 0, 35);
	cout << "[";
	for(int i=0; i<14; i++)
	{
		cout << randomBabble[i]<< ", ";
	}
	cout << randomBabble[15] << "]" << endl;
}

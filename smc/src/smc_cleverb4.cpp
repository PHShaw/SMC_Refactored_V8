
#include "smc_cleverb4.h"

using namespace std;


bool initYarp()
{

	yarp::os::Network yarp;

	target = new Target();

	DMCinPort.open("/smc/in");
	DMCoutPort.open("/smc/out");

	bool ok = false;
	if(!VAM)
	{
		ok = yarp.connect("/target/left/data", "/target/left/read");
		ok &= yarp.connect("/target/right/data", "/target/right/read");
	}
	ok=false;
	cout << "Waiting for connection to dmc" << endl;
	do
	{
		ok =  yarp.connect("/dmc/out", "/smc/in");
		ok &= yarp.connect("/smc/out", "/dmc/in");
		yarp::os::Time::delay(0.2);
	}while(!ok);

	if(VAM)
	{
		VAMinPort.open("/SMCDMC/in");
		VAMoutPort.open("/SMCDMC/out");
//		ok = false;
//		do
//		{
			yarp.connect("/SMCDMC/out","/vamprocess/in");
			yarp.connect("/vamprocess/out","/SMCDMC/in");
//			ok =  yarp.connect("/SMCDMC/out","/vamprocess/in");
//			ok &= yarp.connect("/vamprocess/out","/SMCDMC/in");
//			yarp::os::Time::delay(0.2);
//		}while(!ok);
	}

	return ok;
}

bool init(int argc, char* argv[])
{
	srand( time(NULL));

	yarp::os::Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	yarp::os::Value* val;
	std::string robot;
	if(options.check("robot",val))
	{
		robot = val->asString().c_str();
		cout << "Selected robot: " << robot << endl;
	}
	else
	{
		cout << "A robot can be specified from the command line e.g. --robot [icub|icubSim]+F" << endl;
		robot = "icubSim";
	}

	if(options.check("path",val))
	{
		path = val->asString().c_str();
		cout << "Loading files from path: " << path << endl;

		if(options.check("name",val))
		{
			filename = val->asString().c_str();
			cout << "Loading file set: " << filename << endl;
		}
		else
		{
			cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
			cin >> filename;
		}
	}
	else
	{
		cout << "Enter the path to the directory containing the files: e.g. ../data/ " <<endl;
		cin >> path;
		cout << "Enter the generic name of the files (minus eye/head/GM_ and .xml): e.g. testXV10" << endl;
		cin >> filename;
	}
	target->initLog(path);
	bool load = true;

	heCoor = new handEyeCoordination(load, path, filename);
	GazeMap* gm = heCoor->getGazeMap();
	ehCont = new EyeHeadSaccading(robot, gm, target, SYNCHRONOUS, NEAREST_NEIGHBOUR, load, path, filename, LEARN);
	armReach = new armReaching(robot, gm, LEARN, safe);
	heCoor->init(ehCont, armReach, target, LEARN);
	stm = new ShortTermMemory(target, ehCont);

	ehCont->getEyeController()->verg(5,false);

	if(options.check("blind",val))
	{
		blind = true;
		cout << "\"Shot in the Dark\" mode enabled"<< endl;
	}

	signal(SIGINT, saveAndQuit);

	return true;
}


void saveAndQuit(int param)
{

	target->closeLog();
	ehCont->closeLogs();
	heCoor->closeLogs();

	ehCont->saveMaps();
	heCoor->saveGazeReachMap();


	exit(param);
}





int main(int argc, char* argv[])
{


	//***************************************************************
	// Safe mode selection
	char c;
	cout << "Enable safe mode? y/n" << endl;
	cin >> c;
	if (c == 'y')
		safe = true;
	else if (c == 'n')
		safe = false;
	else
		safe = true;

	cout << "Safe mode is ";
	if (safe)
		cout << "Enabled";
	else
		cout << "DISABLED. PROCEED WITH CAUTION.";
	cout << endl;
	//***************************************************************



	//***************************************************************
	// Initialisation
	bool ok = initYarp();
	ok &= init(argc, argv);
	if(!ok)
	{
		cout << "Errors occurred during initialisation, quitting." << endl;
		return -1;
	}

	ehCont->getEyeController()->verg(10,true);	//approximate range for middle buttons in 8-12, pre-setting vergence to reduce initial vergence time.
	yarp::os::Bottle& start = DMCoutPort.prepare();
	start.addString("start");
	DMCoutPort.write();

	//***************************************************************
	//Remember, DMc eye coordinates are based on 0,0 at centre of the image
	// To DMc: -160, -120
	// From DMc: +160, +120

	bool moved = true;
	bool pressed = false;
	int lastArmCommand = 0;
	pair<int,int> lastEyeCommand (0,0);
	bool rightArmLast;		//Records the arm used for the last reach command.

	string colour;
	bool eyeSuccess=true;

	int objectIndex;

	while(true)
	{
		//Check for new instructions
		//*************************************
		//Read message from DMC, but don't wait

		if(dmcCmd = DMCinPort.read(MSG_WAIT))
		{
			int eyeXCmd,eyeYCmd;
			int armCmd;
			eyeXCmd = dmcCmd->get(0).asInt();
			eyeYCmd = dmcCmd->get(1).asInt();
			armCmd = dmcCmd->get(2).asInt();	//Arm reaches to where eye is looking after saccade.

			cout << "Command received:" << endl;
			cout << "  Eye command: (" << eyeXCmd << ", " << eyeYCmd << ")" << endl;
			cout << "  Arm command: " << armActionStrings[armCmd] << endl;
			cout << "********************************\n" << endl;

			bool eyeChange;
			if(eyeXCmd == lastEyeCommand.first && eyeYCmd == lastEyeCommand.second)
				eyeChange = false;
			else
				eyeChange = true;

			if(eyeChange && !(eyeXCmd==0 && eyeYCmd==0))
			{
				//Deal with the eye command, if !=0,0.
				//Check these coordinates against previously recorded coords
				bool repeat = false;
				/* TODO: Check this recall
				for(int i=0; i<convMem.size(); i++)
				{
					if(eyeXCmd==convMem.at(i)->dmcX && eyeYCmd==convMem.at(i)->dmcY)
					{
						repeat = true;
						ehCont->goToGazeField(convMem.at(i)->gf);
						//TODO: Update vergence?

						colour = convMem.at(i)->colour;

						if(VAM && !blind)
						{
							std::cout<<"Asking to send the object details "<<endl;

							yarp::os::Bottle& VAMout = VAMoutPort.prepare();
							VAMout.clear();
							VAMout.addString("sendobject");
							VAMoutPort.write();

							int counter = 0;
							string dummy;
							vamData = VAMinPort.read(true);
							dummy = vamData->get(counter++).asString();
							std::cout<<"received: "<<dummy<<endl;
							dummy = vamData->get(counter++).asString();
							std::cout<<"received: "<<dummy<<endl;

							int objectIndex = vamData->get(counter++).asInt();
							std::cout<<"received: "<<objectIndex<<endl;

							if(objectIndex == -2) {
								std::cout << "Very few features detected. Please try again" << endl;
								eyeSuccess = false;
								break;
							}

							string subString = colour.substr(2);
							int id = atoi(subString.c_str());
							if(objectIndex != id)
							{
								//VAM thinks this is a different object.
								//SMC is not directly interested in any further info, but will probably store it, just in case.
								dummy = vamData->get(counter++).asString().c_str();		//"featurecount"
								std::cout<<"received: "<<dummy<<endl;
								int featureCount = vamData->get(counter++).asInt();	//n
								std::cout<<"received: "<<featureCount<<endl;

								//Description = "vam"+objectIndex;
								GazeField* gf = ehCont->getGazeField();
								stm->update("vam"+objectIndex, gf);
								object* obj = stm->getObject("vam"+objectIndex);

								for (int i = 0; i<featureCount; i++) {
									pair<int,int> temp;
									temp.first = vamData->get(2*i+counter).asInt();
									temp.second = vamData->get(2*i+counter).asInt();
									std::cout<<" ("<<temp.first<<") ("<<temp.second<<")";
									obj->VAM_featureLocations.push_back(temp);
								}
								std::cout<<endl;
								counter+= featureCount*2;
								dummy = vamData->get(counter++).asString();		//"featurevectors"
								std::cout<<"received: "<<dummy<<endl;

								for (int i = 0; i<featureCount; i++) {
									vector<float> tempVect;
									for(int j = 0; j<40; j++) { //40 is the length of the feature vector
										float temp = vamData->get(counter++).asDouble();
										tempVect.push_back(temp);
										std::cout<<temp<<" ";
									}
									std::cout<<endl;
									obj->VAM_featureVectors.push_back(tempVect);
								}

							}




						}

						stm->update(convMem.at(i)->colour, convMem.at(i)->gf);
						eyeSuccess = true;
						moved = true;
						break;
					}
				}*/

				if(blind && !repeat)
				{
					eyeSuccess = ehCont->fixateEyeHead(eyeXCmd+160, eyeYCmd+120);
//					eyeSuccess = ehCont->fixate(eyeXCmd+160,eyeYCmd+120,
//							eyeXCmd+160,eyeYCmd+120);		//if it returns true, it has moved and is hopefully fixated
																	//if it returns false, it may have moved.

					if(AU){
						cout << "Looking at the success of the eye saccade" << endl;
						eyeSaccading* eyeSac = ehCont->getEyeSaccader();
						if(eyeSac->fr.combiSaccade)
							cout << "A combined eye and head saccade was made" << endl;
						else if(eyeSac->fr.failedToObtainedCurrentEyePosition)
						{
							//no movement will have been made.
							cout << "Unable to currently move the eyes, please try again" << endl;
							moved = false;
						}
						else{
							if(eyeSac->fr.nearestNeighbour)
							{
								cout << "nearest learnt neighbour was used" << endl;
								if(eyeSac->fr.dist>5)
								{
									cout << "and it wasn't very near" << endl;
								}
								cout << "at a distance of " << eyeSac->fr.dist << endl;
							}
							else
							{
								cout << "Followed a direct link" << endl;
							}
							if(eyeSac->fr.unreachable)
							{
								cout << "Tried to make a movement beyond the range of the motors" << endl;
							}
							else
								cout << "and it was reachable" << endl;
							//Compare movement made with desired movement
							//Look for target shortfall from input field
							//Compare with target command?

							double relx, rely;
							relx = eyeSac->fr.endX - eyeSac->fr.startX;
							rely = eyeSac->fr.endY - eyeSac->fr.startY;
							cout << "Relative movement made: " << relx << ", " << rely << endl;

							double ox, oy;
							ox = eyeSac->fr.output->getXcoord();
							oy = eyeSac->fr.output->getYcoord();

							double dist = sqrt((ox-relx)*(ox-relx) + (oy-rely)*(oy-rely));
							cout << "motor difference between desired and actual is: " << dist << endl;

							if(eyeSac->isMotorField(ox-relx, oy-rely))
							{
								cout << "Found a matching motor field for the difference" << endl;
								PolarField* motorDiff = eyeSac->getMotorField(ox-relx, oy-rely);
								if(eyeSac->isLinkedInput(motorDiff))
								{
									cout << "Motor field has linked retina field" << endl;
									PolarField* retinaDiff = eyeSac->getLinkedInput(motorDiff);
									double rdx, rdy;
									rdx = retinaDiff->getXcoord();
									rdy = retinaDiff->getYcoord();
									cout << "It is possible the desired target is near the coordinate: " << rdx << ", " << rdy << endl;
								}
								else
								{
									cout << "Motor field is unlinked" << endl;
								}
	//							cout << "Going to try making the motor difference movement" << endl;
	//							eyeSac->gotoField(motorDiff);
							}
							else
							{
								cout << "No motor field matching difference" << endl;
							}
							moved = true;
						}
					}

				}
				//if not repeat, send it to VAM for analysis
				else if(VAM && !repeat)
				{
					yarp::os::Bottle& VAMout = VAMoutPort.prepare();
					VAMout.clear();
					VAMout.addString("sendvergence");
					VAMout.addString("camera");
					VAMout.addInt(1);	// 0=left cam, 1=right cam
					VAMout.addInt(eyeXCmd+160);
					VAMout.addInt(eyeYCmd+120);
					VAMoutPort.write();
					cout<<"sending coordinates(from dmc) to VAM ("<<(eyeXCmd+160)<<", "<<(eyeYCmd+120)<<")"<< endl;


					//Wait for response from VAM
					vamData = VAMinPort.read(true);
					string dummy;
					dummy = vamData->get(0).asString().c_str();	// should be "vamvergence"
					cout<<"received: "<<dummy  <<endl;
					int xl, yl, xr, yr;
					xl = vamData->get(1).asInt();
					yl = vamData->get(2).asInt();
					xr = vamData->get(3).asInt();
					yr = vamData->get(4).asInt();
					cout << "Received coordinates back from VAM"<< "(" << xl << ", " << yl << ")" <<
																  " (" << xr << ", " << yr << ")" << endl;

					//Note, this will need to have a certain degree of iteration to try to guarantee fixation and vergence
					eyeSuccess = ehCont->fixateEyeHead((xl+xr)/2, (yl+yr)/2);	//should now be fixated
					//now need to verg on target

					/*
					cout << "Attempting to verge on target" << endl;
					bool verged = false;
					do{
						yarp::os::Bottle& VAMout = VAMoutPort.prepare();
						VAMout.clear();
						VAMout.addString("sendvergence");
						VAMout.addString("camera");
						VAMout.addInt(1);	// 0=left cam, 1=right cam
						VAMout.addInt(160);
						VAMout.addInt(120);
						VAMoutPort.write();


						//Wait for response from VAM
						vamData = VAMinPort.read(true);
						string dummy;
						dummy = vamData->get(0).asString().c_str();	// should be "vamvergence"
						cout<<"received: "<<dummy  <<endl;
						int xl, yl, xr, yr;
						xl = vamData->get(1).asInt();
						yl = vamData->get(2).asInt();
						xr = vamData->get(3).asInt();
						yr = vamData->get(4).asInt();
						cout << "Received coordinates back from VAM"<< "(" << xl << ", " << yl << ")" <<
																	  " (" << xr << ", " << yr << ")" << endl;

						verged = ehCont->getEyeSaccader()->verge(xl, yl, xr, yr);
					}while(!verged);	//Should now also be verged on the target
					 */

					//After smc has done vergence, request for object data from VAM
					yarp::os::Bottle& VAMout2 = VAMoutPort.prepare();
					VAMout2.clear();
					VAMout2.addString("sendobject");
					VAMoutPort.write();
					cout << "Requesting object data from VAM" << endl;

					//Wait for response from VAM
					vamData = VAMinPort.read(true);
					int counter = 0;
					dummy = vamData->get(counter++).asString().c_str();		//"vamobject"
					std::cout<<"received: "<<dummy<<endl;
					dummy = vamData->get(counter++).asString().c_str();		//"objectindex"
					std::cout<<"received: "<<dummy<<endl;

					objectIndex = vamData->get(counter++).asInt();	//j
					std::cout<<"received: "<<objectIndex<<endl;

					if(objectIndex == -2) {
						std::cout << "Very few features detected. Please try again" << endl;
						eyeSuccess = false;

					}
					else
					{
						//SMC is not directly interested in any further info, but will probably store it, just in case.
						dummy = vamData->get(counter++).asString().c_str();		//"featurecount"
						std::cout<<"received: "<<dummy<<endl;
						int featureCount = vamData->get(counter++).asInt();	//n
						std::cout<<"received: "<<featureCount<<endl;

						//Description = "vam"+objectIndex;
						GazeField* gf = ehCont->getGazeField();
						stm->update("vam"+objectIndex, gf);
						object* obj = stm->getObject("vam"+objectIndex);

						for (int i = 0; i<featureCount; i++) {
							pair<int,int> temp;
							temp.first = vamData->get(2*i+counter).asInt();
							temp.second = vamData->get(2*i+counter).asInt();
							std::cout<<" ("<<temp.first<<") ("<<temp.second<<")";
							obj->VAM_featureLocations.push_back(temp);
						}
						std::cout<<endl;
						counter+= featureCount*2;
						dummy = vamData->get(counter++).asString();		//"featurevectors"
						std::cout<<"received: "<<dummy<<endl;

						for (int i = 0; i<featureCount; i++) {
							vector<float> tempVect;
							for(int j = 0; j<40; j++) { //40 is the length of the feature vector
								float temp = vamData->get(counter++).asDouble();
								tempVect.push_back(temp);
								std::cout<<temp<<" ";
							}
							std::cout<<endl;
							obj->VAM_featureVectors.push_back(tempVect);
						}
						eyeSuccess = true;
					}
					moved = true;

				}
				else if(!repeat)	//and not VAM
				{
					double targX, targY, dist;
					colour = target->getNearestTo(eyeXCmd+160, eyeYCmd+120, &targX, &targY, &dist);
					ehCont->fixate(targX, targY, colour, false);
					double depth;
					ehCont->verge(colour, &depth, true);

					//Update memory of object.
					GazeField* gf = ehCont->getGazeField();
					stm->update(colour, gf);
					moved = true;
					eyeSuccess = true;
				}

				lastEyeCommand = pair<int,int> (eyeXCmd, eyeYCmd);

			}



			if(armCmd!=lastArmCommand)
				pressed=false;

			bool armSuccess;

			if(armCmd != DO_NOTHING)
			{
				armController* ac = armReach->getArmController();
				if(armCmd==ARM_GO_HOME)
				{
					ac->armsToRest();
				}
				else if(armCmd==ARM_REACH_AND_PRESS)
				{
					GazeField* gf = ehCont->getGazeField();
					ReachField* rf = new ReachField();
					double dist;
					heCoor->getNearestReachForGaze(gf, &rf, &dist);
					if(gf->getXcoord()==0 && gf->getYcoord()==0)
						armSuccess=false;
					else
					{
						bool rightArm;
						ReachConfig* rc = rf->getPreferredReach(&rightArm);
						if(rc==new ReachConfig())
						{
							//failed to locate an arm config with with to reach and press in appropriate direction.
							armSuccess=false;
							rc = rf->getLeftReach();
							rightArm = false;
						}
						else
						{
							if(rightArm != rightArmLast)
							{
								//diff arm used, therefore need to move other arm to home pose first
								armReach->toRest(rightArmLast);	//threaded
							}
							//check if arm is already in the correct position, in which case, just press
							double* armConf = new double[16];
							rc->toArray(armConf);
							bool reached = armReach->getArmController()->isAtPosition(armConf,rightArm);

							if(reached)
							{
								//arm already in position
								ac->armAction(ac->PRESS,armConf,rightArm);
							}
							else if(eyeChange || armCmd != lastArmCommand)
							{
								if(rightArm)
									ac->armAction(ac->REACH_AND_PRESS_RIGHT,armConf,rightArm);
								else
									ac->armAction(ac->REACH_AND_PRESS_LEFT,armConf,rightArm);
							}
							if(dist<16)	armSuccess=true;
							else armSuccess=false;
							rightArmLast = rightArm;
							moved = true;
						}
					}

				}	//End arm reach and press where eye is
				else if(armCmd == ARM_REACH_AND_PUSH_LEFT)
				{
					//Can only push left with the right arm!
					GazeField* gf = ehCont->getGazeField();
					ReachField* rf = new ReachField();
					double dist;
					heCoor->getNearestReachForGaze(gf, &rf, &dist);
					if(gf->getXcoord()==0 && gf->getYcoord()==0)
						armSuccess=false;
					else
					{
						bool rightArm;
						ReachConfig* rc = rf->getRightReach();
						bool recheck =false;
						if(rc==new ReachConfig())
						{
							//failed to locate an arm config with with to reach and press in appropriate direction.
							armSuccess=false;
							rc = rf->getLeftReach();
							rightArm = false;
							recheck=true;
						}
						else
						{
							if(dist<16)	armSuccess=true;
							else armSuccess=false;
							rightArm = true;
						}

						if(recheck && rc==new ReachConfig())
						{
							//failed to locate an arm config with with to reach and press in appropriate direction.
							armSuccess=false;
						}
						else
						{
							if(rightArm != rightArmLast)
							{
								//diff arm used, therefore need to move other arm to home pose first
								armReach->toRest(rightArmLast);	//threaded
							}
							//check if arm is already in the correct position, in which case, just press
							double* armConf = new double[16];
							rc->toArray(armConf);
							bool reached = armReach->getArmController()->isAtPosition(armConf,rightArm);

							if(reached && rightArm)
							{
								//arm already in position
								ac->armAction(ac->SMALL_PUSH,armConf,rightArm);
							}
							else if(eyeChange || armCmd != lastArmCommand)
							{
								if(rightArm)
									ac->armAction(ac->REACH_AND_PUSH_LEFT,armConf,rightArm);
								else
									ac->armAction(ac->REACH_RIGHT,armConf,rightArm);
							}
							rightArmLast = rightArm;
							moved = true;
						}
					}

				}	//End arm reach and push left where eye is
				else if(armCmd == ARM_REACH_AND_PUSH_RIGHT)
				{
					//Can only push RIGHT with the LEFT arm!
					GazeField* gf = ehCont->getGazeField();
					ReachField* rf = new ReachField();
					double dist;
					heCoor->getNearestReachForGaze(gf, &rf, &dist);
					if(gf->getXcoord()==0 && gf->getYcoord()==0)
						armSuccess=false;
					else
					{
						bool rightArm;
						ReachConfig* rc = rf->getLeftReach();
						bool recheck = false;
						if(rc==new ReachConfig())
						{
							//failed to locate an arm config with with to reach and press in appropriate direction.
							armSuccess=false;
							rc = rf->getRightReach();
							rightArm = true;
							recheck = true;
						}
						else
						{
							if(dist<16)	armSuccess=true;
							else armSuccess=false;
							rightArm = false;
						}

						if(recheck && rc==new ReachConfig())
						{
							//failed to locate an arm config with with to reach and press in appropriate direction.
							armSuccess=false;
						}
						else
						{
							if(rightArm != rightArmLast)
							{
								//diff arm used, therefore need to move other arm to home pose first
								armReach->toRest(rightArmLast);	//threaded
							}
							//check if arm is already in the correct position, in which case, just press
							double* armConf = new double[16];
							rc->toArray(armConf);
							bool reached = armReach->getArmController()->isAtPosition(armConf,rightArm);

							if(reached && rightArm)
							{
								//arm already in position
								ac->armAction(ac->SMALL_PUSH,armConf,rightArm);
							}
							else if(eyeChange || armCmd != lastArmCommand)
							{
								if(rightArm)
									ac->armAction(ac->REACH_AND_PUSH_RIGHT,armConf,rightArm);
								else
									ac->armAction(ac->REACH_LEFT,armConf,rightArm);
							}
							rightArmLast = rightArm;
							moved = true;
						}
					}

				}	//End arm reach and push right where eye is
				else
				{
					heCoor->armRest();
					moved = true;

				}

				lastArmCommand = armCmd;
			}	//End processing arm command
		}

		int armState;
		//check to see if the arm is moving
		if(!heCoor->getArmController()->armsStationary())	//If arms are not stationary, mark the arm state as moving
		{
			//Arm still moving

			armState = ARM_MOVING;
			moved = true;
		}
		else if(lastArmCommand != DO_NOTHING)	//If arms are not moving, and last arm command isn't do nothing, mark arm state as action complete
		{
			armState = ARM_ACTION_COMPLETE;
			//if( !armSuccess) armState = ARM_FAILED;
		}
		else	//if arms are not moving and last arm command was an instruction, mark arm state as action complete anyway.
		{
			armState = ARM_ACTION_COMPLETE;
			moved = true;
		}


		if(moved)	//Send update to CNR with current arm state
		{
			yarp::os::Bottle& DMCout = DMCoutPort.prepare();
			DMCout.clear();

			if(eyeSuccess)
				DMCout.addString("saccadeComplete");
			else
				DMCout.addString("saccadeFailed");

			int memSize = stm->size();
			DMCout.addInt(memSize);

			multimap<object*, GazeField*> mem = stm->getObjectMemory();
			for(multimap<object*, GazeField*>::iterator it = mem.begin();
					it!=mem.end();
					++it)
			{


				//convert gaze field motor values into coordinates.
				GazeField* gf = it->second;
				float x,y, dist;
				x = eyeXtoHeadX(gf->getXcoord());
				y = eyeYtoHeadY(gf->getYcoord());
				printf("Gaze field (%.2f, %.2f)\n",x,y);

				//Use head map to look up retina points for greater range.

				if(x<-50 || x>55 || y<-50 || y>38)
				{
					//coords likely to fall outside the mapped area of the retina
					cout << "A coordinate falls outside the mapped area, attempting to adjust" << endl;

					int adjX=0, adjY=0;
					if(x < -50)
					{
						// ~retina x 315 -- motor x -40, retina x 300 -- motor x 40
						adjX = (int)((x/-50)*5 + ((x/-50) * 315))-320;
						x = (int)x%-50;
						cout << "Adjusted -x to adjX: " << adjX << " and x: " << x << endl;
					}
					else if(x>56)
					{
						adjX = (int)((x/56)*3 + ((x/56) * -317))+320;
						x = (int)x%56;
						cout << "Adjusted +x to adjX: " << adjX << " and x: " << x << endl;
					}

					if(y < -50)
					{
						// ~retina y 220 -- motor y -40, retina y 20 -- motor y 40
						adjY = (int)((y/-50)*3 + ((y/-50) * 237))-240;
						y = (int)y%-50;
						cout << "Adjusted -y to adjY: " << adjY << " and y: " << y << endl;
					}
					else if(y>38)
					{
						adjY = (int)((y/38)*3 + ((y/38) * -237))+240;
						y = (int)y%38;
						cout << "Adjusted +y to adjY: " << adjY << " and y: " << y << endl;
					}
					PolarField* motor = ehCont->getHeadSaccader()->getNearestLearntOutput(x,y,&dist);
					PolarField* retina = ehCont->getHeadSaccader()->getLinkedInput(motor);
					if(retina->getXcoord()==0 && retina->getYcoord()==0)
					{
						//failed to get a valid retina field... what should I do here?
						cout << "erm, failed to get a valid retina field..." << endl;
					}
					else
					{
						adjX += retina->getXcoord();
						adjY += retina->getYcoord();
						DMCout.addInt(it->first->id);
						DMCout.addInt((int)(adjX-160));
						DMCout.addInt((int)(adjY-120));
						DMCout.addInt(it->first->fixateCount);

						if(it->first->fixateCount == 1)
							convMem.push_back(new dmcCoord((int)(adjX-160), (int)(adjY-120), gf, colour));

					}

				}
				else
				{
					PolarField* motor = ehCont->getHeadSaccader()->getNearestLearntOutput(x,y,&dist);
					PolarField* retina = ehCont->getHeadSaccader()->getLinkedInput(motor);
					if(retina->getXcoord()==0 && retina->getYcoord()==0)
					{
						//failed to get a valid retina field... this should never happen...
					}
					else
					{
						DMCout.addInt(it->first->id);
						DMCout.addInt((int)(retina->getXcoord()-160));
						DMCout.addInt((int)(retina->getYcoord()-120));
						DMCout.addInt(it->first->fixateCount);

						if(it->first->fixateCount == 1)
							convMem.push_back(new dmcCoord((int)(retina->getXcoord()-160), (int)(retina->getYcoord()-120), gf, colour));

					}
				}

			}
			DMCout.addInt(armState);

			if(VAM && !blind)
			{
				// Last vamData bottle should contain the response to a "send object" request.
				DMCout.addString("vamobject");
				DMCout.addString("objectindex");
				if(vamData->get(2).asInt()==-2)
				{
					DMCout.addInt(-2);
					DMCout.addString("featurecount");
					DMCout.addInt(0);
				}
				else
				{
					for(int i=2; i<vamData->size(); i++)
					{
						DMCout.add(vamData->get(i));
					}

				}

			}
			else
			{
				DMCout.addString("vamobject");
				DMCout.addString("objectindex");
//				int objectIndex = 1;//replace 1 by o/SMCDMC/object index
				DMCout.addInt(objectIndex);
				std::cout<<"object index "<< objectIndex << endl;

				DMCout.addString("featurecount");
				int featureCount = 100;//change 100 to number of features on the object
				std::cout<<"feature count "<<featureCount<<endl;
				DMCout.addInt(featureCount);

				DMCout.addString("featurelocations");
				std::cout<<"sending feature locations"<<endl;
				for (int i = 0; i<featureCount; i++) {
					int xV = rand()%320;//replace it by feature location X coordinate
					int yV = rand()%240;//replace it by feature location Y coordinate
					DMCout.addInt(xV);
					DMCout.addInt(yV);
//					std::cout<<"("<<xV<<","<<yV<<"), ";
				}
//				std::cout << endl;

				DMCout.addString("featurevectors");
				std::cout<<"sending feature vectors"<<endl;
				for (int i = 0; i<featureCount; i++) {
					for(int j = 0; j<40; j++) { //40 is the lenght of the feature vector
						float featVect = (float)rand()/(float)RAND_MAX;
//						std::cout<<featVect<<" ";
						DMCout.addDouble(featVect);//change rand() to feature vector values
					}
//					std::cout<<endl;
				}
			}

			DMCoutPort.write();
		}

		moved = false;
	}

}




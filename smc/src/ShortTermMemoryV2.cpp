/*
 * ShortTermMemory.cpp
 *
 *  Created on: 27 Apr 2012
 *      Author: icub
 */

#include "ShortTermMemoryV2.h"

using namespace yarp::os;

std::ostream& operator <<(std::ostream& outs, const object& source)
{
	outs << "Object: {" << source.description << " at depth " << source.depth << ", excitation: " <<
			source.excitation << "}";

	return outs;
}

bool operator ==(const object& f1, const object& f2)
{
	return f1.description.compare(f2.description)==0;
}
bool operator <(const object& o1, const object& o2)
{
	return o1.description < o2.description;		// must return a constant comparison, even when the object moves
}

bool compare(object* o1, object* o2)
{
	return o1->excitation > o2->excitation;
}


//ShortTermMemory::ShortTermMemory(Target* t, EyeHeadController* eh, torsoSaccading* ts)
//{
//	target = t;
//	ehCont = eh;
//	torSac = ts;
//
//	reachDepthThreshold = 80;
//
//	objIDcounter_smc = 1;
//
//	Network yarp;
//	gazed.open("/gaze/live/out");
//	yarp.connect("/gaze/live/out", "/gaze/live/in");
//
//}


ShortTermMemory::ShortTermMemory(Target* t, EyeHeadSaccading* eh)
{
	target = t;
	ehCont = eh;

	reachDepthThreshold = 80;
	objIDcounter_smc = 1;

	Network yarp;
	gazed.open("/gaze/live/out");
	yarp.connect("/gaze/live/out", "/gaze/live/in");
}

ShortTermMemory::~ShortTermMemory()
{
	fieldToObjectMemory.clear();
	objectToFieldMemory.clear();
}

/**
 * Only update the STM when an object is fixated.  Give the description of this object, along with
 * the gaze field in which this object falls.
 */
void ShortTermMemory::update(string description, GazeField* gf)
{
	bool found = false;
	for(multimap<object*, GazeField*>::iterator it = objectToFieldMemory.begin();
			it!=objectToFieldMemory.end();
			++it)
	{
		if(it->first->description.compare(description)==0)	//previously encountered object
		{
			it->first->fixateCount ++;
			found = true;
			it->second = gf;	//Naively updating this in case it has moved at all.
		}
	}

	if(!found)
	{
		object* o = new object(description, 35, 100, objIDcounter_smc);		//temporary default values
		objIDcounter_smc ++;
		objectToFieldMemory.insert(pair<object*,GazeField*>(o,gf));
		fieldToObjectMemory.insert(pair<GazeField*,object*>(gf,o));
	}

}

object* ShortTermMemory::getObject(std::string description)
{
	bool found = false;
	for(multimap<object*, GazeField*>::iterator it = objectToFieldMemory.begin();
			it!=objectToFieldMemory.end();
			++it)
	{
		if(it->first->description.compare(description)==0)	//previously encountered object
		{
			found = true;
			return it->first;
		}
	}

	if(!found)
	{
		object* o = new object(description, 35, 100, -2);		//temporary default values
	}
}


/**
 * Needs to look for targets that have moved and targets that are now obscured,
 * or that have moved slightly and now have another target close to where the
 * previous target was.  In this case, the targets should be reported as a stack,
 * unless the target obscuring the previous target is the hand.
 *
 *
 */
void ShortTermMemory::update(int holdingObjectID)
{
	Bottle* bottle = target->getAllTargets();

	for(multimap<object*, GazeField*>::iterator it = objectToFieldMemory.begin();
			it!=objectToFieldMemory.end();
			++it)
	{
		it->first->disappeared = true;
		it->first->moved = false;
		it->first->report = false;
		it->first->occluded = false;
		it->first->stacked = false;
	}

	int size = bottle->size();
	if(size == 0)
	{
		Time::delay(0.5);
		bottle = target->getAllTargets();
		size = bottle->size();
	}

	if(size>0)
	{
		int numObjs = size/elements;
		string colour;
		double targX, targY, depth;
		int size;

		for(int i=0; i<numObjs; i++)
		{
			colour = bottle->get(i*elements).asString();
			targX  = bottle->get(i*elements+1).asDouble();
			targY  = bottle->get(i*elements+2).asDouble();
			size   = bottle->get(i*elements+3).asInt();

			if(colour == "")
				continue;

			ehCont->getDepth(colour, &depth);

			bool known = false;
			object* o = new object(colour, depth, size, objIDcounter_smc);
			objIDcounter_smc++;
			GazeField* gf_old;
			//NOTE: multimap.find(o) doesn't seem to work! Always returns multimap.end();
			if(objectToFieldMemory.size()>0)
			{
				for(multimap<object*, GazeField*>::iterator it=objectToFieldMemory.begin();
						it!=objectToFieldMemory.end(); ++it)
				{
					printf("Checking: %s\n",it->first->description.c_str());
					if(colourToId(it->first->description.c_str()) == colourToId(colour.c_str()))
					{
						o = it->first;
						gf_old = it->second;
						known = true;
						printf("Found known match\n");
						break;
					}
				}

			}




			if(!known)
			{
				GazeField* gf = new GazeField();
				bool success = fixate(o, &gf);

				if(success)
				{
					objectToFieldMemory.insert(pair<object*,GazeField*>(o,gf));
					fieldToObjectMemory.insert(pair<GazeField*,object*>(gf,o));

					paint(o, gf);
				}
				else
				{
					printf("Not fixated on %s target\n", colour.c_str());
				}
			}
			else if(colourToId(colour.c_str()) == holdingObjectID)
			{
				//This is the object that we are holding, so therefore want to update it with the hands new
				//position, which we might not know yet

				o->disappeared=false;
				o->occluded = true;
			}
			else	//Previously remembered target
			{
				//Target has previously been recorded, so now to check if it has moved / obscured
				o->disappeared=false;

				//Check to see if the size of the object has drastically changed, possibly indicating an occlusion
				if(size < o->size*OCCLUSION_THRESHOLD && colour.compare(HAND)!=0)
				{
					//Object has significantly reduced in size and is not the hand...
					//Assuming the object hasn't actually moved
					printf("Target %s has probably been occluded by something!\n",colour.c_str());
					o->occluded = true;
				}
				else
				{

					//get gaze direction to object
					double* headMotorConf2 = new double[6];
					bool success = ehCont->getGazeDirection(targX, targY, headMotorConf2);

					bool retest = true;

					if(success)
					{
						GazeField* gf = ehCont->getGazeMap()->getGazeField(headMotorConf2);

						if(gf->getXcoord()!= 0 && gf->getYcoord()!=0)
						{
							double x1,x2,y1,y2;
							gf->getGazeDirection(&x1, &y1);
							gf_old->getGazeDirection(&x2, &y2);
							double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

							printf("Visible dist change: %.2f\n", dist);
							if(dist<MOVEMENT_THRESHOLD)
							{
								//Assuming target hasn't moved, and as we already have a fixated gaze field don't want to change this.
								o->excitation --;
								retest = false;
							}
						}

						if(!retest)
							continue;
					}


					//need to fixate to check and get accurate update on position
					GazeField* gf_new = new GazeField();
					double dist;
					reFixate(o, gf_old, &gf_new, &dist);

					//Fixate on target to add accurate gazeField, then return to starting pose
					if(dist<MOVEMENT_THRESHOLD)
					{
						//Not 'actually' moved
					}
					else
					{
						repaint(o, gf_old, gf_new, size);

						o->update(depth, size, true);	//also resets excitation to initial level
						changeLink(o, gf_old, gf_new);
					}

				}//Target not occluded

			}//Previously remembered object
		}//Looping through list of currently visible targets
	}
	else
	{
		printf("No objects visible\n");
	}



	for(multimap<object*,GazeField*>::iterator it = objectToFieldMemory.begin();
			it != objectToFieldMemory.end(); ++it)
	{
		//Check to sort out holding object to gaze field here as well
		if(holdingObjectID>0 && colourToId(it->first->description.c_str())==holdingObjectID)
		{
			it->first->disappeared = false;	//just in case object isn't visible in the hand
			it->first->occluded = true;

			if(objectToFieldMemory.size()>0)
			{
				for(multimap<object*, GazeField*>::iterator ret=objectToFieldMemory.begin();
						ret!=objectToFieldMemory.end(); ++ret)
				{
					printf("Checking: %s\n",ret->first->description.c_str());
					if(colourToId(ret->first->description.c_str()) == colourToId("yellow"))
					{
						changeLink(it->first, it->second, ret->second);
						repaint(it->first, it->second, ret->second, it->first->size);
						break;
					}
				}

			}
		}


		if(it->first->disappeared)
		{
			printf("I can no longer see the %s target\n", it->first->description.c_str());

			//is it fully obscured by another object?
			if(fieldToObjectMemory.count(it->second) > 1)
			{
				//most likely that the object is fully obscured, although only want to know if hand is obscuring it
				pair<multimap<GazeField*, object*>::iterator, multimap<GazeField*, object*>::iterator> ret;
				ret = fieldToObjectMemory.equal_range(it->second);
				for(multimap<GazeField*,object*>::iterator jt = ret.first; jt != ret.second; ++jt)
				{
					//TODO: Make this more general? whilst also considering that some objects may really disappear, and not just be obscured!
					if(colourToId(jt->second->description.c_str()) == colourToId("hand") && !jt->second->disappeared)
					{
						printf("Think the %s target is obscured by the hand\n", it->first->description.c_str());
						it->first->disappeared = true;
						it->first->occluded = true;
						break;
					}
				}
			}

			//should it have been visible?
			double gazeX, gazeY;
			ehCont->getGazeDirection(&gazeX, &gazeY);
			if(abs(it->second->getXcoord() - gazeX)>60 || abs(it->second->getYcoord() - gazeY)>35)
			{
				printf("But I think it is out of view, so it's okay\n");
				it->first->disappeared = false;
				it->first->excitation -= 2;
			}
			else
			{
				printf("and I think it should still have been in view... oh dear!\n");
				it->first->excitation +=5;
//				it->first->report = true;
			}
		}


	}


	//TODO: This goes into an infinite loop?
	//Is anything stacked? Remember not to include the hand when trying to identify a stack.  Also, stacking is currently identified by schemas
//	for(multimap<GazeField*, object*>::iterator it = fieldToObjectMemory.begin();
//			it!=fieldToObjectMemory.end(); ++it)
//	{
//
//		if(fieldToObjectMemory.count(it->first) > 1)
//		{
//			printf("Found a gazefield with %i objects in it:\n", (int)fieldToObjectMemory.count(it->first));
//			pair<multimap<GazeField*, object*>::iterator, multimap<GazeField*, object*>::iterator> ret;
//			ret = fieldToObjectMemory.equal_range(it->first);
//			for(multimap<GazeField*,object*>::iterator jt = ret.first; jt != ret.second; ++jt)
//			{
//				printf("%s\n",jt->second->description.c_str());
//				jt->second->stacked = true;
//			}
//			it= ret.second;
//		}
//	}


	//================================================================================ OKAY TO HERE I THINK


	bottle->clear();


}

object* ShortTermMemory::getTarget()
{
	object* best = NULL;
	int excitation = 0;
	for(multimap<object*, GazeField*>::iterator it = objectToFieldMemory.begin();
			it != objectToFieldMemory.end(); ++it)
	{
		if(it->first->excitation > excitation)
		{
			best = it->first;
			excitation = best->excitation;
		}
	}

	return best;	//first element in list has highest excitation
}


void ShortTermMemory::printMemory()
{
	printf("There are %i objects in memory at this time:\n",objectToFieldMemory.size());
	for(multimap<object*, GazeField*>::iterator it = objectToFieldMemory.begin();
			it != objectToFieldMemory.end(); ++it)
	{
		object* o = it->first;
		GazeField* gt = it->second;
		cout << "  " << *o << ", at gaze field: " << *gt << endl;

	}


}

void ShortTermMemory::reportMemory(Bottle *reply, int holdingObject)
{
	for(multimap<object*, GazeField*>::iterator it = objectToFieldMemory.begin();
				it != objectToFieldMemory.end(); ++it)
	{
		object* o = it->first;
		GazeField* gf = it->second;

		int id = colourToId(o->description.c_str());

		if(!o->disappeared || o->report || o->occluded )
		{
			if(gf->wx!=0 && gf->wy!=0 && gf->wz!=0)
			{
				reply->addString("visual");
				reply->addInt(id);
				reply->addString(o->description.c_str());
	//			double gazeX, gazeY;
	//			gf->getGazeDirection(&gazeX, &gazeY);

				reply->addDouble(gf->wx);
				reply->addDouble(gf->wy);
				reply->addDouble(gf->wz);
				reply->addInt(o->size);
				reply->addInt(o->excitation);

				if(gf->wx > -240 || gf->wx < -340)
					reply->addInt(1);		//predicted to be out of reach
				else							//vice versa
					reply->addInt(0);		//within reach


//			if(o->stacked)				//Not yet agreed extra information here
//				reply->addInt(1);
//			else
//				reply->addInt(0);

				printf("adding visual %i %s %.2f %.2f %.2f %i %i\n", id,o->description.c_str(),gf->wx, gf->wy, gf->wz, o->size, o->excitation);
			}
		}
	}
}

void ShortTermMemory::getVisualTouch(Bottle* reply)
{
	object* o = new object("yellow",0,0);
	GazeField* gf;
	multimap<object*, GazeField*>::iterator it = objectToFieldMemory.find(o);
	bool found = false;

	if(objectToFieldMemory.size()>0)
	{
		for(multimap<object*, GazeField*>::iterator it=objectToFieldMemory.begin();
				it!=objectToFieldMemory.end(); ++it)
		{
			printf("Checking for touch: %s\n",it->first->description.c_str());
			if(colourToId(it->first->description.c_str()) == colourToId(o->description.c_str())  && !it->first->disappeared)
			{
				o = it->first;
				gf = it->second;
				found = true;
				printf("Found hand\n");
				break;
			}
		}

	}


	if(found)
	{
		if(fieldToObjectMemory.count(gf) > 1)
		{
			pair<multimap<GazeField*, object*>::iterator, multimap<GazeField*, object*>::iterator> ret =
				fieldToObjectMemory.equal_range(gf);

			for(multimap<GazeField*, object*>::iterator jt = ret.first; jt!=ret.second; ++jt)
			{
				//Check not hand again
				//check if within reach depth or not
				if(jt->second != o)	//not hand
				{
					if(jt->first->wx > -240 || jt->first->wx < -340)
					{
						reply->addString("touch");
						reply->addInt(colourToId(jt->second->description.c_str()));
						reply->addInt(jt->second->excitation);
						printf("I think I am touching, hand: %i, object: %i\n",jt->second->depth,o->depth);
					}
					else
					{
						reply->addString("point");
						reply->addInt(colourToId(jt->second->description.c_str()));
						reply->addDouble(jt->second->excitation);
						printf("I think I am only pointing, hand: %i, object: %i\n",jt->second->depth,o->depth);
					}
				}
			}
		}

	}
}


int ShortTermMemory::getNewHoldingID()
{
	object* o = new object("yellow",0,0);
	GazeField* gf;
	multimap<object*, GazeField*>::iterator it = objectToFieldMemory.find(o);
	if(it!=objectToFieldMemory.end())
	{
		o = it->first;
		gf = it->second;
		if(fieldToObjectMemory.count(gf) > 1)
		{
			pair<multimap<GazeField*, object*>::iterator, multimap<GazeField*, object*>::iterator> ret =
				fieldToObjectMemory.equal_range(gf);

			for(multimap<GazeField*, object*>::iterator jt = ret.first; jt!=ret.second; ++jt)
			{
				//Check not hand again
				//check if within reach depth or not
				if(jt->second != o)	//not hand
				{
					return colourToId(jt->second->description.c_str());
				}
			}
		}

	}

	return -1;

}


void ShortTermMemory::updateHandPosition(GazeField* gf)
{
	pair<multimap<GazeField*, object*>::iterator, multimap<GazeField*, object*>::iterator> ret =
					fieldToObjectMemory.equal_range(gf);

	for(multimap<GazeField*, object*>::iterator jt = ret.first; jt!=ret.second; ++jt)
	{
		if(colourToId(jt->second->description.c_str()) == 2)
		{
			Bottle& gazeOld = gazed.prepare();
			gazeOld.clear();
			int radius = (int)((double)(jt->first->getRadius())*((double)(jt->second->size)/100.0));
			gazeOld.addInt(jt->first->getXcoord());
			gazeOld.addInt(jt->first->getYcoord());
			gazeOld.addInt(jt->first->getRadius());
			gazeOld.addString("gray");
			gazed.write();

			changeLink(jt->second, jt->first, gf);

			Bottle& gaze = gazed.prepare();
			gaze.clear();
			radius = (int)((double)(gf->getRadius())*((double)(jt->second->size)/100.0));
			gaze.addInt(gf->getXcoord());
			gaze.addInt(gf->getYcoord());
			gaze.addInt(gf->getRadius());
			gaze.addString(jt->second->description.c_str());
			gazed.write();
			break;
		}
	}

}


GazeField* ShortTermMemory::getHandGaze()
{
	object* o = new object("yellow",0,0);
	multimap<object*, GazeField*>::iterator it = objectToFieldMemory.find(o);
	if(it!= objectToFieldMemory.end())
		return it->second;
	else
		return new GazeField();
}


//*************************** PRIVATE FUNCTIONS***********************************//

bool ShortTermMemory::fixate(object* o, GazeField** gf)
{
	string colour = o->description;
	double targX, targY, depth;
	target->getTarget(&targX, &targY, colour);
	bool success = true;

	//Record current position so that we can return here later if necessary.
	double* startPose = new double[6];
	ehCont->getHeadController()->getCurrentPosition(startPose);
	HeadConfig* startConfig = new HeadConfig(startPose);
	//Fixate on target to add accurate gazeField, then return to starting pose
	ehCont->fixate(targX,targY,colour, true);
	ehCont->autoCenter(colour);
	ehCont->centreEyesInHead();
	Time::delay(0.2);
	ehCont->autoCenter(colour);
	Time::delay(0.2);
	if(target->targetCentred(&targX, &targY, colour))
	{
		ehCont->getDepth(colour, &depth);
		o->depth = depth;
		GazeField* gf_temp = ehCont->getGazeField();
		if(gf_temp->getXcoord()!= 0 && gf_temp->getYcoord()!=0)
		{
			*gf = gf_temp;
		}
		else
		{
			printf("Invalid gaze field obtained for %s target\n", colour.c_str());
			success = false;
		}
	}
	else
	{
		printf("Not fixated on %s target\n", colour.c_str());
		success = false;
	}
	ehCont->goToHeadConfig(startConfig);

	return success;
}


bool ShortTermMemory::reFixate(object* o, GazeField* gf_old, GazeField** gf_new, double* dist)
{
	string colour = o->description;
	double targX, targY, depth;
	target->getTarget(&targX, &targY, colour);
	bool success = true;

	//Record current position so that we can return here later if necessary.
	double* startPose = new double[6];
	ehCont->getHeadController()->getCurrentPosition(startPose);
	HeadConfig* startConfig = new HeadConfig(startPose);


	//Fixate on target to add accurate gazeField, then return to starting pose
	ehCont->fixate(targX,targY,colour, true);
	ehCont->autoCenter(colour);
	ehCont->centreEyesInHead();
	Time::delay(0.2);
	ehCont->autoCenter(colour);
	Time::delay(0.2);
	if(target->targetCentred(&targX, &targY, colour))
	{
		ehCont->getDepth(colour, &depth);
		o->depth = depth;
		GazeField* gf = ehCont->getGazeField();
		if(gf->getXcoord()!= 0 && gf->getYcoord()!=0)
		{
			*gf_new = gf;
			double x1,x2,y1,y2;
			gf->getGazeDirection(&x1, &y1);
			gf_old->getGazeDirection(&x2, &y2);
			*dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

		}
		else
		{
			printf("Invalid gaze field obtained for %s target\n", colour.c_str());
			success = false;
		}
	}
	else
	{
		printf("Not fixated on %s target\n", colour.c_str());
		success = false;
	}
	ehCont->goToHeadConfig(startConfig);

	return success;
}


void ShortTermMemory::changeLink(object* o, GazeField* gf_old, GazeField* gf_new)
{
	//Update gaze field
	pair<multimap<object*, GazeField*>::iterator,
		 multimap<object*, GazeField*>::iterator> ret = objectToFieldMemory.equal_range(o);
	for(multimap<object*, GazeField*>::iterator jt = ret.first; jt!= ret.second;
			++jt)
	{
		if(jt->second==gf_old)
		{
			objectToFieldMemory.erase(jt);
			break;
		}
	}
	objectToFieldMemory.insert(pair<object*, GazeField*>(o, gf_new));

	//update gaze to object mapping
	pair<multimap<GazeField*, object*>::iterator,
		 multimap<GazeField*, object*>::iterator> ret2 = fieldToObjectMemory.equal_range(gf_old);
	for(multimap<GazeField*, object*>::iterator jt = ret2.first; jt!=ret2.second; ++jt)
	{
		if(jt->second==o)
		{
			fieldToObjectMemory.erase(jt);
			break;
		}
	}
	fieldToObjectMemory.insert(pair<GazeField*, object*>(gf_new, o));
}


void ShortTermMemory::repaint(object* o, GazeField* gf_old, GazeField* gf_new, int newSize)
{
	Bottle& wipe = gazed.prepare();
	wipe.clear();
	int radius = (int)((double)(gf_old->getRadius())*((double)(o->size)/1000.0));
	wipe.addInt(gf_old->getXcoord());
	wipe.addInt(gf_old->getYcoord());
	wipe.addInt(gf_old->getRadius());
	wipe.addString("gray");

	gazed.write();

	Time::delay(0.01);

	Bottle& gaze = gazed.prepare();
	gaze.clear();
	radius = (int)((double)(gf_new->getRadius())*((double)(newSize)/1000.0));
	gaze.addInt(gf_new->getXcoord());
	gaze.addInt(gf_new->getYcoord());
	gaze.addInt(gf_new->getRadius());
	gaze.addString(o->description.c_str());

	gazed.write();

}

void ShortTermMemory::paint(object* o, GazeField* gf)
{
	Bottle& gaze = gazed.prepare();
	gaze.clear();
	int radius = (int)((double)(gf->getRadius())*((double)(o->size)/1000.0));
	gaze.addInt(gf->getXcoord());
	gaze.addInt(gf->getYcoord());
	gaze.addInt(gf->getRadius());
	gaze.addString(o->description.c_str());

	gazed.write();

}


int ShortTermMemory::colourToId(const char *colour) {
	if (strcmp(colour, "red") == 0)		return 1;
	if (strcmp(colour, "yellow") == 0)	return 2;
	if (strcmp(colour, "hand") == 0)	return 2;
	if (strcmp(colour, "green") == 0)	return 3;
	if (strcmp(colour, "blue") == 0)	return 4;
	if (strcmp(colour, "white") == 0)	return 5;
	else return 0;
}

string ShortTermMemory::idToColour(const int id) {
	if (id==1) return "red";
	if (id==2) return "yellow";
	if (id==3) return "green";
	if (id==4) return "blue";
	if (id==5) return "white";
	else return "unknown";
}

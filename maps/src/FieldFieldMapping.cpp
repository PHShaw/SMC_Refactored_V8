/*
 * FILE: FieldFieldMapping.cpp
 * CLASS IMPLEMENTED: FieldFieldMapping (see FieldFieldMapping.h for documentation)
 *
 *  Created on: 5 Oct 2010
 *      Author: phs
 */
//#define DEBUG


#include "FieldFieldMapping.h"
//#include <algorithm>	// Provides copy function
#include <math.h>		// Provides abs, cos, pow, sin, sqrt functions
#include <vector>

#ifdef DEBUG
#include <iostream>
#endif

using namespace std;


bool operator==(const FieldLink& l1, const FieldLink& l2)
{
	return (l1.input==l2.input && l1.output==l2.output);
}
std::ostream& operator <<(std::ostream& outs, const FieldLink& source)
{
	outs << "Link: [" << *(source.input) << "|" << *(source.output) << "] strength: " << source.usage;
	return outs;
}

FieldFieldMapping::FieldFieldMapping() {
	init(0, 0, 0, 0, 0,
			0, 0, 0, 0, 0);

}
FieldFieldMapping::FieldFieldMapping(int pInputFieldType, float pInputResX, float pInputResY,
						int pOutputFieldType, float pOutputResX, float pOutputResY)
{
	init(pInputFieldType, 0, pInputResX, 0, pInputResY,
				pOutputFieldType, 0, pOutputResX, 0, pOutputResY);
}


FieldFieldMapping::FieldFieldMapping(int inputFieldType, float inputMinX, float inputMaxX,
											float inputMinY, float inputMaxY,
						int outputFieldType, float outputMinX, float outputMaxX,
											float outputMinY, float outputMaxY)
{
	init(inputFieldType, inputMinX, inputMaxX, inputMinY, inputMaxY,
			outputFieldType, outputMinX, outputMaxX, outputMinY, outputMaxY);


}




FieldFieldMapping::~FieldFieldMapping() {
#ifdef DEBUG
	cout << "Deleting input fields..." << endl;
#endif
//	delete inputFields;
	inputFields.clear();
	inputFields.resize(1);
#ifdef DEBUG
	cout << "Deleting output fields..." << endl;
#endif
//	delete outputFields;
	outputFields.clear();
	outputFields.resize(1);
#ifdef DEBUG
	cout << "Deleting linked fields..." << endl;
#endif
//	delete linkedFields;
//	linkedFields.clear();
//	linkedFields.resize(1);
	links.clear();
	links.resize(1);
}


Field* FieldFieldMapping:: getInputField(size_t ID)
{
	return getField(ID, &inputFields, numInputFields);
}
Field* FieldFieldMapping:: getInputField(float pX, float pY)
{
	Field* input = getField(pX,pY, &inputFields, numInputFields);
	if(inputFieldType==DYNAMIC && input == nullField)
	{
		input = new PolarField(inputFields.size(), pX, pY, DYNAMIC_RADIUS);
		inputFields.push_back(input);
		numInputFields = inputFields.size();
	}
	else if(inputFieldType==POLAR_MAP && input == nullField)
	{
		float dist;
		PolarField* f = getNearestLearntInputField(pX, pY, &dist);
		if(dist<100)
			input = new PolarField(inputFields.size(), pX, pY, f->getRadius());
		else
			input = new PolarField(inputFields.size(), pX, pY, 4);

	}

	return input;
}
PolarField* FieldFieldMapping:: getNearestLearntInputField(float pX, float pY, float* distance)
{
//	return getNearestLearntField(pX,pY, &inputFields, numInputFields, distance);


	float shortestDist = 1000000.f;
	PolarField *closest;
	for(unsigned int i=0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		PolarField* tField = (PolarField*)l->input;
		//Take the hypotenus of the distance from x,y to the centre of the field
		float fX = tField->getXcoord();
		float fY = tField->getYcoord();
		float hypo = sqrt((pX - fX)*(pX - fX) + (pY - fY)*(pY - fY));

		if(hypo<shortestDist)
		{
			shortestDist = hypo;
			closest = tField;
		}
	}
	*distance = shortestDist;
	return closest;

}
PolarField* FieldFieldMapping:: getNearestReliableLearntInputField(float pX, float pY, float* distance)
{
//	return getNearestLearntField(pX,pY, &inputFields, numInputFields, distance);


	float shortestDist = 1000000.f;
	PolarField *closest;
	for(unsigned int i=0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		PolarField* tField = (PolarField*)l->input;
		//Take the hypotenus of the distance from x,y to the centre of the field
		float fX = tField->getXcoord();
		float fY = tField->getYcoord();
		float hypo = sqrt((pX - fX)*(pX - fX) + (pY - fY)*(pY - fY));

		if(hypo<shortestDist && l->getUsage()>=-1)
		{
			shortestDist = hypo;
			closest = tField;
		}
	}
	*distance = shortestDist;
	return closest;

}
vector<Field**> FieldFieldMapping::getLearntInputFields(size_t* numLearntFields)
{
	//return getLearntFields(numLearntFields, &inputFields, numInputFields);
}


Field* FieldFieldMapping:: getOutputField(size_t ID)
{
	return getField(ID, &outputFields, numOutputFields);
}
Field* FieldFieldMapping:: getOutputField(float pX, float pY)
{
	// for new dynamic point fields, if returns null field, generate a new field at the coordinates and push it onto the list.
	Field* output = getField(pX,pY, &outputFields, numOutputFields);
	if(outputFieldType==DYNAMIC && output == nullField)
	{
		output = new PolarField(outputFields.size(), pX, pY, DYNAMIC_RADIUS);
		outputFields.push_back(output);
		numOutputFields = outputFields.size();
	}
	else if(inputFieldType==POLAR_MAP && output == nullField)
	{
		float dist;
		PolarField* f = getNearestLearntOutputField(pX, pY, &dist);
		if(dist<100)
			output = new PolarField(outputFields.size(), pX, pY, f->getRadius());
		else
			output = new PolarField(outputFields.size(), pX, pY, 2);
	}

	return output;
}
PolarField* FieldFieldMapping:: getNearestLearntOutputField(float pX, float pY, float* distance)
{
//	return getNearestLearntField(pX,pY, &outputFields, numOutputFields, distance);

	float shortestDist = 1000000.f;
	PolarField *closest;
	for(unsigned int i=0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		PolarField* tField = (PolarField*)l->output;
		//Take the hypotenus of the distance from x,y to the centre of the field
		float fX = tField->getXcoord();
		float fY = tField->getYcoord();
		float hypo = sqrt((pX - fX)*(pX - fX) + (pY - fY)*(pY - fY));

		if(hypo<shortestDist)
		{
			shortestDist = hypo;
			closest = tField;
		}
	}
	*distance = shortestDist;
	return closest;
}

PolarField* FieldFieldMapping:: getNearestReliableLearntOutputField(float pX, float pY, float* distance)
{
//	return getNearestLearntField(pX,pY, &outputFields, numOutputFields, distance);

	float shortestDist = 1000000.f;
	PolarField *closest;
	for(unsigned int i=0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		PolarField* tField = (PolarField*)l->output;
		//Take the hypotenus of the distance from x,y to the centre of the field
		float fX = tField->getXcoord();
		float fY = tField->getYcoord();
		float hypo = sqrt((pX - fX)*(pX - fX) + (pY - fY)*(pY - fY));

		if(hypo<shortestDist && l->getUsage()>=-1)	//TODO: more sophisticated reliability to dist pref needed?
		{
			shortestDist = hypo;
			closest = tField;
		}
	}
	*distance = shortestDist;
	return closest;
}

vector<Field**> FieldFieldMapping::getLearntOutputFields(size_t *numLearntFields)
{
	return getLearntFields(numLearntFields, &outputFields, numOutputFields);
}

//NOTE: WHEN ADDING A LINK, NEED TO TELL THE FIELDS THAT THEY HAVE BEEN LEARNT...
bool FieldFieldMapping:: addLink(Field *inputField, Field *outputField)
{
#ifdef DEBUG
	cout << "In addLink(Field*, Field*)" << endl;
	cout << "There are currently "<< links.size() << " links with a capacity for lots of links."<<endl;
#endif
	if(*inputField == *nullField || *outputField == *nullField)
		return false;

	inputField->linkAdded();
	outputField->linkAdded();
//	if((numLinks*2+2)>linkCapacity)
//	{
//#ifdef DEBUG
//		cout << "Expanding link capacity."<<endl;
//#endif
//		//expand list
//		//if fail to expand list, return false or exception.
////		polarField** largerlist = new polarField*[numLinks*2+2];
////		copy(linkedFields, linkedFields+numLinks*2, largerlist);
////		delete [] linkedFields;
////		linkedFields = largerlist;
//		linkedFields.reserve(linkCapacity+2);
//		linkCapacity += 2;
//
//	}

//	linkedFields[numLinks*2] = &inputField;
//	linkedFields[(numLinks*2)+1] = &outputField;

	if(containsLink(inputField,outputField))
	{
#ifdef DEBUG
		cout << "Link already exists, using field" << endl;
#endif
		FieldLink* l = getLink(inputField, outputField);
		l->useField();
#ifdef DEBUG
		cout << "Link updated" << endl;
#endif
	}
	else
	{
#ifdef DEBUG
		cout << "Adding new link to list" << endl;
#endif
		links.push_back(new FieldLink(inputField, outputField));
#ifdef DEBUG
		cout << "Link added" << endl;
#endif
	}
//	linkedFields.push_back(inputField);
//	linkedFields.push_back(outputField);

//	cout << **linkedFields[numLinks*2] << endl;
//	cout << linkedFields.size()<<endl;
//	numLinks++;

	return true;
}
bool FieldFieldMapping:: addLink(size_t inputID, size_t outputID)
{
	if(checkID(inputID,true) && checkID(outputID,false))
	{
		Field *tInputField, *tOutputField;
		tInputField = getInputField(inputID);
		tOutputField = getOutputField(outputID);

		if(*tInputField==*nullField || *tOutputField==*nullField)
			return false;	// unable to find one of the fields so no link established.
		else
			return addLink(tInputField, tOutputField);
	}
	else
		return false;
}
bool FieldFieldMapping:: addLink(float inputX, float inputY, float outputX, float outputY)
{
	if(checkCoords(inputX, inputY, true) && checkCoords(outputX, outputY, false))
	{
		Field *tInputField, *tOutputField;
		tInputField = getInputField(inputX, inputY);
		tOutputField = getOutputField(outputX, outputY);

		if(*tInputField==*nullField || *tOutputField==*nullField)
			return false;	// unable to find one of the fields so no link established.
		else
			return addLink(tInputField, tOutputField);
	}
	else
		return false;
}


bool FieldFieldMapping::addLink(Field *inputField, Field *outputField, unsigned int calculatedSteps)
{
	bool success = addLink(inputField, outputField);
	if(success)
	{
		inputField->setCalcdLinks(calculatedSteps);
		outputField->setCalcdLinks(calculatedSteps);
	}
	return success;
}
bool FieldFieldMapping::addLink(size_t inputID, size_t outputID, unsigned int calculatedSteps)
{
	if(checkID(inputID,true) && checkID(outputID,false))
	{

		Field *tInputField, *tOutputField;
		tInputField = getInputField(inputID);
		tOutputField = getOutputField(outputID);
		return addLink(tInputField, tOutputField, calculatedSteps);
	}
	else
		return false;
}
bool FieldFieldMapping::addLink(float inputX, float inputY, float outputX, float outputY, unsigned int calculatedSteps)
{
#ifdef DEBUG
	cout << "Checking coordinates are valid" << endl;
#endif
	if(checkCoords(inputX, inputY, true) && checkCoords(outputX, outputY, false))
	{
#ifdef DEBUG
		cout << "Coordinates are valid, now checking to see if field already exists" <<endl;
#endif
		Field *tInputField, *tOutputField;
		tInputField = getInputField(inputX, inputY);
		tOutputField = getOutputField(outputX, outputY);
#ifdef DEBUG
		cout << "Coordinates are valid, now looking up fields" <<endl;
#endif

		if(*tInputField==*nullField || *tOutputField==*nullField)
		{
#ifdef DEBUG
			cout << "One or both of the fields was not found" << endl;
#endif
			return false;	// unable to find one of the fields so no link established.
		}
		else
		{
#ifdef DEBUG
			cout << "Fields located, now attempting to add a link" << endl;
#endif
			return addLink(tInputField, tOutputField, calculatedSteps);
		}
	}
	else
	{
#ifdef DEBUG
		cout << "Coordinates were not valid" << endl;
#endif
		return false;
	}
}


bool FieldFieldMapping::containsLink(Field *inputField, Field *outputField)
{
	for(size_t i = 0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		if(*(l->input)==*inputField && *(l->output)==*outputField)
		{
			return true;
		}
	}

//	for(unsigned int i=0; i<linkedFields.size(); i+=2)
//	{
//		if(linkedFields[i]==inputField && linkedFields[i+1]==outputField)
//		{
//			return true;
//		}
//	}
	return false;
}

FieldLink* FieldFieldMapping::getLink(Field* inputField, Field* outputField)
{
	for(size_t i = 0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		if(*(l->input)==*inputField && *(l->output)==*outputField)
		{
			return l;
		}
	}
	return 0;
}

/*
 * // erase the 6th element
  myvector.erase (myvector.begin()+5);

  // erase the first 3 elements:
  myvector.erase (myvector.begin(),myvector.begin()+3);
 *
 */
bool FieldFieldMapping::deleteLink(Field *inputField, Field *outputField) // Also change learnt status of fields to 0
{
	//for(unsigned int i=0; i<numLinks; i++)

	for(size_t i = 0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		if(*(l->input)==*inputField && *(l->output)==*outputField)
		{
			links.erase(links.begin()+i);
//			numLinks--;
			inputField->setCalcdLinks(Field::UNLEARNT);
			outputField->setCalcdLinks(Field::UNLEARNT);
			return true;
		}
	}

//	for(unsigned int i=0; i<linkedFields.size(); i+=2)
//	{
//		if(linkedFields[i] == inputField && linkedFields[i+1]==outputField)
//		{
//			cout << *linkedFields[i] << endl;
//			cout << *linkedFields[i+1] << endl;
//			linkedFields.erase(linkedFields.begin()+i, linkedFields.begin()+i+2); //removes up to, but not including last elements
////			for(int j=i*2; j<numLinks*2-2; j++)
////			{
////				linkedFields[j] = linkedFields[j+2];
////			}
////			linkedFields.resize(numLinks*2-2);
//			numLinks --;
//			linkCapacity = numLinks*2;
//
//
//
//			inputField->setCalcdLinks(Field::UNLEARNT);
//			outputField->setCalcdLinks(Field::UNLEARNT);
//			return true;
//		}
//	}
	return false;
}
bool FieldFieldMapping::deleteLink(size_t inputID, size_t outputID)
{
	if(checkID(inputID,true) && checkID(outputID,false))
	{
		Field *tInputField, *tOutputField;
		tInputField = getInputField(inputID);
		tOutputField = getOutputField(outputID);
		return deleteLink(tInputField, tOutputField);
	}
	else
		return false;
}

bool FieldFieldMapping::changeLink(Field *inputField, Field *oldOutputField, Field *newOutputField, unsigned int calculatedSteps) // Also change learnt status of old output field to 0
{

	for(size_t i = 0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		if(*(l->input)==*inputField && *(l->output)==*oldOutputField)
		{
			oldOutputField->setCalcdLinks(Field::UNLEARNT);
			l->output = newOutputField;
			newOutputField->setCalcdLinks(calculatedSteps);
		}
	}

//	for(unsigned int i=0; i<numLinks; i++)
//	{
//		if(*linkedFields[i*2] == *inputField && *linkedFields[i*2+1]==*oldOutputField)
//		{
////			linkedFields.erase(linkedFields.begin()+i*2+1);
////			linkedFields.insert(linkedFields.begin()+i*2+1,newOutputField);
//			linkedFields[i*2+1] = newOutputField;
//			oldOutputField->setCalcdLinks(Field::UNLEARNT);
//			newOutputField->setCalcdLinks(calculatedSteps);
//		}
//	}
}


/*
Field* FieldFieldMapping:: getLinkedOutput(Field* inputField) const
{

	for (int i=0; i<numLinks; i++)
	{
		if(*inputField == *linkedFields[i*2])
		{
			return linkedFields[i*2+1];
		}
	}
	return nullField;
}*/

Field* FieldFieldMapping:: getLinkedOutput(Field* inputField) const
{	//look for strongest linked output, if there is more than one.

	Field* strongest = nullField;
	int matchCounter = 0;
	FieldLink* strongLink;


	for(size_t i = 0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		if(*(l->input)==*inputField)
		{
			if(!matchCounter)
			{
				strongLink = l;
				matchCounter ++;
				strongest = l->output;
			}
			else
			{
				matchCounter ++;
				if(l->usage > strongLink->usage)
				{
					strongLink = l;
					strongest = l->output;
				}
			}
		}
	}


//	for (int i=0; i<numLinks; i++)
//	{
//		if(*inputField == *linkedFields[i*2])
//		{
//			if(strongest == nullField)
//			{
//				strongest = linkedFields[i*2+1];
//				matchCounter ++;
//			}
//			else
//			{
//				Field* temp = linkedFields[i*2+1];
//				matchCounter++;
//				if(temp->getUsage() > strongest->getUsage())
//				{
//					strongest = linkedFields[i*2+1];
//				}
//			}
//		}
//	}

#ifdef DEBUG
	printf("There were %i matching links found, with strongest being strength %i\n", matchCounter, strongest->getUsage());
#endif

	return strongest;
}



Field* FieldFieldMapping:: getLinkedInput(Field* outputField) const
{
	Field* strongest = nullField;
	int matchCounter = 0;
	FieldLink* strongLink;
	bool setLink = false;


	for(size_t i = 0; i<links.size(); i++)
	{
		FieldLink* l = links.at(i);
		if(*(l->output)==*outputField)
		{
			if(!matchCounter)
			{
				strongLink = l;
				matchCounter ++;
				strongest = l->input;
			}
			else
			{
				matchCounter ++;
				if(l->usage > strongLink->usage)
				{
					strongLink = l;
					strongest = l->input;
				}
			}
		}
	}

//	for (int i=0; i<numLinks; i++)
//	{
//		if(*outputField == *linkedFields[i*2+1])
//		{
//			if(strongest == nullField)
//			{
//				strongest = linkedFields[i*2];
//				matchCounter ++;
//			}
//			else
//			{
//				Field* temp = linkedFields[i*2];
//				matchCounter++;
//				if(temp->getUsage()> strongest->getUsage())
//				{
//					strongest = linkedFields[i*2];
//				}
//			}
////			return linkedFields[i*2];
//		}
//	}
	return strongest;
}

void FieldFieldMapping:: resetLinks()
{
	links.clear();

//	linkedFields.clear();
////	linkedFields = new polarField*[linkCapacity];
//	linkedFields.reserve(linkCapacity);
//	numLinks = 0;
}

int FieldFieldMapping:: unlearntInputFields() const
{
	int counter = 0;
	int learnt = 0;
	cout << "There are " << inputFields.size() << " input fields" << endl;
	for(unsigned int i=0; i<inputFields.size(); i++)
	{
		Field* f = inputFields.at(i);
		if(f->isLearnt())
			learnt++;
		else
			counter ++;
	}
	cout << "There are " << learnt << " learnt input fields and " << counter << " unlearnt input fields" << endl;

	return counter;
}

int FieldFieldMapping:: unlearntOutputFields() const
{
	int counter = 0;
	for(unsigned int i=0; i<outputFields.size(); i++)
	{
		Field* f = outputFields.at(i);
		if(!f->isLearnt())
			counter ++;
	}

	return counter;
}


int FieldFieldMapping::getNumGoodLinks() const
{
	int counter = 0;

	for(size_t i = 0; i<links.size(); i++)
	{
		if(links.at(i)->usage > 0)
			counter ++;
	}

	return counter;
}


void FieldFieldMapping:: printStats()
{
	cout << "Type of input map: " << inputFieldType << endl;
	cout << "Type of output map: " << outputFieldType << endl;
	cout << "There are " << numInputFields << " input fields, with " << unlearntInputFields() << " unlearnt fields" << endl;
	cout << "There are " << numOutputFields << " output fields, with " << unlearntOutputFields() <<" unlearnt fields" << endl;
	cout << "There are " << links.size() << " learnt links" << endl;

#ifdef ARGH
	cout << "INPUT FIELDS:"<<endl;
	//for(size_t i = 0; i<numInputFields; i++)
	for(unsigned int i = 0; i<inputFields.size(); i++)
	{
//		cout << inputFields[i] << endl;
		cout << *inputFields.at(i) << endl;
	}


	cout << "OUTPUT FIELDS:"<<endl;
//	for(size_t i = 0; i<numOutputFields; i++)
	for(unsigned int i = 0; i<outputFields.size(); i++)
	{
//		cout << outputFields[i] << endl;
		cout << *outputFields.at(i) << endl;
	}
#endif
}

void FieldFieldMapping:: printLinkedFields()
{
//	if(*inputField == **linkedFields[i*2])
//			return *linkedFields[i*2+1];

	if(links.size() == 0)
	{
		cout << "No links available" << endl;
		return;
	}
	printf("There are %i linked fields\n", links.size());
//	cout << linkedFields.size() << endl;
	for(size_t i=0; i<links.size(); i++)
	{
		cout << "There is a link between ";

		Field* inputField = links.at(i)->input;	//linkedFields[i*2];
		Field* outputField = links.at(i)->output; //linkedFields[i*2+1];
		switch (inputFieldType)
		{
		case POLAR_MAP:	//defined as const int in ffm
		{	PolarField* pfield = (PolarField*)inputField;
			cout << "polar field: ID: "<< pfield->getIndex() << " " << *pfield << " and ";
			break;}
		case METRIC_MAP:
		{	metricField* mfield = (metricField*)inputField;
			cout << "metric field: ID: "<< mfield->getIndex() << " " << *mfield << " and ";
			break;}
		default:
		{	//Field* field = **linkedFields[i*2];
//			cout << "unknown type of field: ID: "<< *inputField->getIndex() << " " << **inputField << " and ";
		}}
		switch (outputFieldType)
		{
		case POLAR_MAP:	//defined as const int in ffm
		{	PolarField* pfield = (PolarField*)outputField;
			cout << "polar field: ID: "<< pfield->getIndex() << " " << *pfield << endl;
			break;}
		case METRIC_MAP:
		{	metricField* mfield = (metricField*)outputField;
			cout << "metric field: ID: "<< mfield->getIndex() << " " << *mfield << endl;
			break;}
		default:
		{	//Field* field = **linkedFields[i*2+1];
//			cout << "unknown type of field: ID: "<< *outputField->getIndex() << " " << **outputField << endl;
		}}
	}
}



//*******************************************
// PRIVATE METHODS
//*******************************************


void FieldFieldMapping::init(int pInputFieldType, float pInputMinX, float pInputMaxX,
											float pInputMinY, float pInputMaxY,
						int pOutputFieldType, float pOutputMinX, float pOutputMaxX,
											float pOutputMinY, float pOutputMaxY)
{
#ifdef DEBUG
	cout << "initialising mapping parameters..." << endl;
#endif
	inputFieldType =  pInputFieldType;
	inputMinX = pInputMinX;
	inputMaxX = pInputMaxX;
	inputMinY = pInputMinY;
	inputMaxY = pInputMaxY;

	outputFieldType = pOutputFieldType;
	outputMinX = pOutputMinX;
	outputMaxX = pOutputMaxX;
	outputMinY = pOutputMinY;
	outputMaxY = pOutputMaxY;

#ifdef DEBUG
	cout << "evaluating mapping type..." << endl;
#endif
	//Initialise input vector list
	if(inputFieldType == POLAR_MAP)
	{
		inputCapacity = 500;
#ifdef DEBUG
		cout << "initiating input polarField store..." << endl;
#endif
//		inputFields = new polarField[inputCapacity];
		inputFields.reserve(inputCapacity);

		numInputFields=0;
#ifdef DEBUG
		cout << "generating polar fields..." << endl;
#endif
		makePolarFields(inputMinX, inputMaxX, inputMinY, inputMaxY, &numInputFields, &inputFields, &inputCapacity);
	}
	else if(inputFieldType == METRIC_MAP)
	{
		inputCapacity = 9;
//		inputFields = new metricField[inputCapacity];
		inputFields.reserve(inputCapacity);
	}
	else if(inputFieldType == DYNAMIC)
	{
		inputCapacity = 500;
		numInputFields=0;
	}
	else
	{
		inputCapacity = 5;
//		inputFields = new polarField[inputCapacity];
		inputFields.reserve(inputCapacity);
	}




	//Initialise output vector list
	if(outputFieldType == POLAR_MAP)
	{
		outputCapacity = 500;
//		outputFields = new polarField[outputCapacity];
		outputFields.reserve(outputCapacity);
		numOutputFields=0;
		makePolarFields(outputMinX, outputMaxX, outputMinY, outputMaxY, &numOutputFields, &outputFields, &outputCapacity);
	}
	else if(outputFieldType == METRIC_MAP)
	{
		outputCapacity = 9;
//		outputFields = new metricField[outputCapacity];
		outputFields.reserve(outputCapacity);
	}
	else if(outputFieldType == DYNAMIC)
	{
		outputCapacity = 500;
		numOutputFields=0;
	}
	else
	{
		outputCapacity=5;
//		outputFields = new polarField[outputCapacity];
		outputFields.reserve(outputCapacity);
	}

//	linkCapacity = 300;
//	linkedFields = new polarField*[linkCapacity];
//	linkedFields.reserve(linkCapacity);
//	numLinks = 0;
	nullField = new Field();
}



void FieldFieldMapping :: makePolarFields(float minX, float maxX, float minY, float maxY, size_t *numFields, vector<Field*> *fields, size_t *fieldCapacity)
//Uses math.h
{
#ifdef DEBUG
	cout << "Calculating map proportions" << endl;
#endif
	float proportionHorizontal = (abs((int)minX)+abs((int)maxX))/320.0f;
	float proportionVertical = (abs((int)minY)+abs((int)maxY))/240.0f;

	float proportion =0.f;
	if(proportionHorizontal>proportionVertical)
		proportion = proportionHorizontal;
	else
		proportion = proportionVertical;


	float radiusCompensation = 1.02*pow(proportion,-0.78);

	float centreX = (minX+maxX)/2;
	float centreY = (minY+maxY)/2;

//	cout << "Starting to calculate individual fields..." << endl;
	//cout << motorProportion << endl;
	size_t index = 0;
	for(int i=0; i<lines-1; i++)
	{
		for(int j=1; j<rings; j++)	// Starting from the 5th ring gives a fovea of diameter ~30.9 pixels for resolution 324x244 (now starting from 4th)
		{
			float angle = 0.f;
			if(j%2 == 0)
			{
				angle = (float)i/lines*360;
			//	cout << "even" << endl;
			}
			else
			{
				angle = ((float)i+0.5f)/lines*360;
			//	cout << "odd" << endl;
			}

			float distance = rings*omega*j*pow(1.013,j)/10;
			distance *= proportion;
			float radius = 3+(360/rings)*pow(distance,0.975)*0.5*PI*overlap/180;
			radius *= proportion * radiusCompensation;



			float c_x = centreX + distance*cos(toRadians(angle));
			float c_y = centreY + distance*sin(toRadians(angle));

			if(c_x > minX && c_x < maxX && c_y > minY && c_y < maxY)		//limit it to just the rings that fall within the motor range.
			{
				//cout << index << endl;



				c_x = (float)((int)(c_x*100))/100.f;
				c_y = (float)((int)(c_y*100))/100.f;
				radius = (float)((int)(radius*10))/10.f;
#ifdef ARGH
				cout << index << ": " << c_x << ", " << c_y << ", " << radius << endl;
#endif

				if(index >= *fieldCapacity)
				{
//					for(int i=0; i<index; i++)
//					{
//						cout << fields[i] << endl;
//					}
#ifdef DEBUG
					cout << "Expanding polarField store" << endl;
#endif
//					polarField* largerList = new polarField[*fieldCapacity+5];
//					copy (fields,fields+*numFields,largerList);
//					delete [] fields;
//					fields = largerList;
					fields->reserve(*fieldCapacity+5);
					*fieldCapacity += 5;

//					for(int i=0; i<index; i++)
//					{
//						cout << fields[i] << endl;
//					}
				}

//				fields[index] = new polarField(index,c_x, c_y, radius);
				fields->push_back(new PolarField(index,c_x, c_y, radius));
				//fields->push_back(polarField(index,c_x, c_y, radius));
				index++;
				//cout << c_x << ", " << c_y << ", " << radius << endl;
			}

		}
	}
	*numFields = index;
	//cout << index << endl;
	//return fields;

}


Field* FieldFieldMapping::getField(size_t ID, vector<Field*> *fields, size_t numFields) const
{
	Field* tField;
	for(size_t i=0; i < numFields; i++)
	{
		tField = fields->at(i);
		if (tField->getIndex() == ID)
		{
			return tField;
		}
	}
//	Field* nullField;
	return nullField;
}
Field* FieldFieldMapping:: getField(float pX, float pY, vector<Field*> *fields, size_t numFields) const
{
	PolarField* tField;
	PolarField* closest;
	float shortestDist = 1000000.f;
	for(size_t i=0; i < numFields; i++)
	{
		tField = (PolarField*)fields->at(i);

		//Take the hypotenus of the distance from x,y to the centre of the field
		float fX = tField->getXcoord();
		float fY = tField->getYcoord();
		float hypo = sqrt((pX - fX)*(pX - fX) + (pY - fY)*(pY - fY));

		//compare the distance to the radius to see if the point falls inside the field
		float radius = tField->getRadius();
		if(hypo<radius)
		{
			if (hypo < shortestDist)
			{
				shortestDist = hypo;
				closest = tField;
			}
		}
	}
	if(shortestDist != 1000000.f)
		return closest;
	else
		return nullField;
}

//to find nearest metric field, either need to move coordinates to central point, or calculate central point
//Note this should probably be based on links, rather than the isLearnt parameter
PolarField* FieldFieldMapping:: getNearestLearntField(float pX, float pY, vector<Field*> *fields, size_t numFields, float* distance) const
{
//	polarField *tField;
	float shortestDist = 1000000.f;
	PolarField *closest;
#ifdef DEBUG
	cout << "There are " << fields->size() << " fields to look through" << endl;
#endif
	//for(size_t i=0; i < numFields; i++)
	for(unsigned int i=0; i<fields->size(); i++)
	{
//		tField = (polarField*)fields->at(i);
#ifdef ARGH
		cout << *(PolarField*)fields->at(i) << endl;
#endif
		if(fields->at(i)->isLearnt())
		{
#ifdef DEBUG
			cout << "Found a learnt field!  " << *(PolarField*)fields->at(i) << endl;
#endif
			//Take the hypotenus of the distance from x,y to the centre of the field
			float fX = fields->at(i)->getXcoord();
			float fY = fields->at(i)->getYcoord();
			float hypo = sqrt((pX - fX)*(pX - fX) + (pY - fY)*(pY - fY));

			if(hypo<shortestDist)
			{
				shortestDist = hypo;
				closest = (PolarField*)fields->at(i);
#ifdef DEBUG
				cout << "and its closer" << endl;
#endif
			}
		}
	}
#ifdef DEBUG
	cout << "Returning field: " << *closest << endl;
#endif
	*distance = shortestDist;
	return closest;
}



vector<Field**> FieldFieldMapping::getLearntFields(size_t* numLearntFields, vector<Field*> *fields, size_t numFields)
{
//	polarField **learntFields = new polarField*[10];
	size_t learnt = 0;
	size_t capacity = 2;
	vector<Field**> learntFields(capacity);
	for(size_t i=0; i < numFields; i++)
	{
//		Field* tField = fields->at(i);
		if(fields->at(i)->isLearnt())
		{
			if(learnt >= capacity)
			{
//				polarField** largerlist = new polarField*[capacity+1];
//				copy(learntFields, learntFields+learnt, largerlist);
//				delete [] learntFields;
//				learntFields = largerlist;
				learntFields.reserve(capacity+1);
				capacity ++;
			}
			learntFields[learnt] = &fields->at(i);
			learnt ++;
		}

	}
	*numLearntFields = learnt;
	return learntFields;
}


float FieldFieldMapping:: toRadians(float degrees) const
{
	return PI/180.0f * degrees;
}


bool FieldFieldMapping:: checkCoords(float x, float y, bool isInput) const
{
	if(isInput)
	{
		return  x < inputMaxX && x > inputMinX &&
				y < inputMaxY && y > inputMinY;
	}
	else
	{
		return  x < outputMaxX && x > outputMinX &&
				y < outputMaxY && y > outputMinY;
	}
}

//Return true if valid ID
bool FieldFieldMapping:: checkID(unsigned int ID, bool isInput) const
{
	if(isInput)
	{
		return ID < numInputFields;
	}
	else
	{
		return ID < numOutputFields;
	}
}



// In main method, typedef ffm name

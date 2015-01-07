/*
 * fieldExtract.cpp
 * This program takes an xml file of links and extracts the field coordinates, radii and usage
 * for both input and output links and writes them to an input and output text file.
 * These can then be loaded into matlab to display the learnt fields and frequency of usage.
 *
 *  Created on: 9 Dec 2010
 *      Author: icub
 */

#include <fstream>
#include <vector>

#include "FieldFieldMapping.h"

using namespace std;
typedef FieldFieldMapping ffm;

int main()
{
	ofstream inputFields;
	ofstream outputFields;


	inputFields.open("../data/fields/allInputFields.txt");
	inputFields << "x y radius\n";

	outputFields.open("../data/fields/allOutputFields.txt");
	outputFields << "x y radius\n";

	ffm* ppm = new ffm(POLAR_MAP, 0.f, 324.f, 0.f, 244.f, POLAR_MAP, -60.f, 60.f, -53.f, 53.f);


	size_t numFields;
	vector<Field*> inputFieldList = ppm->getAllInputFields(&numFields);
	for(size_t i=0; i<inputFieldList.size(); i++)
	{
		PolarField* inputField = (PolarField*) inputFieldList.at(i);
		inputFields << inputField->getXcoord() << " " << inputField->getYcoord() << " "
							<< inputField->getRadius() << "\n";
	}

	vector<Field*> outputFieldList = ppm->getAllOutputFields(&numFields);
	for(size_t i=0; i<outputFieldList.size(); i++)
	{
		PolarField* outputField = (PolarField*) outputFieldList.at(i);
		outputFields << outputField->getXcoord() << " " << outputField->getYcoord() << " "
							<< outputField->getRadius() << "\n";
	}



	inputFields.close();
	outputFields.close();
	delete ppm;
	return (0);

}

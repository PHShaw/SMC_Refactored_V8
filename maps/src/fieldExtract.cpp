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

#include "FFM_IO.h"

int main(int argc, char* argv[])
{
	ofstream inputFields;
	ofstream outputFields;

	ofstream combined;

	ffm* ppm;
	FFM_IO io;

	string path, filename;
	if(argc == 2)
	{
		path = string(argv[1]);
		cout << "loading files from path: " << path << endl;
		cout << "Assuming default filename: testXV10" <<endl;
		filename = "testXV10";
	}
	else if(argc >= 3)
	{
		path = string(argv[1]);
		cout << "loading files from path: " << path << endl;
		filename = string(argv[2]);
		cout << "loading files from filename: " << filename << endl;
	}
	else
	{
		cout << "Enter the path to load eye and head links from:" <<endl;
		cin >> path;
		cout << "Enter the generic filename to load eye and head links from:" <<endl;
		cin >> filename;
	}

	ppm = io.loadMappingFromXML(path + "eye_" + filename + ".xml");
//	ppm->printLinkedFields();

	int numLinks = ppm->getNumLinks();
	vector<FieldLink*> linkedFields = ppm->getLinkedFields();

	string fullname = path + "eye_inputFields.txt";
	inputFields.open(fullname.c_str());
	inputFields << "x y radius usage\n";

	fullname = path + "eye_outputFields.txt";
	outputFields.open(fullname.c_str());
	outputFields << "x y radius usage\n";

	fullname = path + "eye_links.txt";
	combined.open(fullname.c_str());
	combined << "inpX inpY inpRad outpX outpY outpRad lnkUsg\n";


	for(size_t i=0; i<numLinks; i++)
	{
		PolarField* inputField = (PolarField*)(linkedFields.at(i)->input);
		PolarField* outputField = (PolarField*)(linkedFields.at(i)->output);

		inputFields << inputField->getXcoord() << " " << inputField->getYcoord() << " "
					<< inputField->getRadius() << " " << inputField->getUsage() << "\n";

		outputFields << outputField->getXcoord() << " " << outputField->getYcoord() << " "
					<< outputField->getRadius() << " " << outputField->getUsage() << "\n";

		combined <<  inputField->getXcoord() << " " << inputField->getYcoord() << " " << inputField->getRadius() << " "
				 << outputField->getXcoord() << " " << outputField->getYcoord() << " " << outputField->getRadius() << " "
				 << linkedFields.at(i)->getUsage() << "\n";
	}

	inputFields.close();
	outputFields.close();
	combined.close();


	ppm = io.loadMappingFromXML(path + "head_" + filename + ".xml");
//	ppm->printLinkedFields();

	numLinks = ppm->getNumLinks();
	linkedFields = ppm->getLinkedFields();

	fullname = path + "head_inputFields.txt";
	inputFields.open(fullname.c_str());
	inputFields << "x y radius usage\n";

	fullname = path + "head_outputFields.txt";
	outputFields.open(fullname.c_str());
	outputFields << "x y radius usage\n";

	fullname = path + "head_links.txt";
	combined.open(fullname.c_str());
	combined << "inpX inpY inpRad outpX outpY outpRad lnkUsg\n";


	for(size_t i=0; i<numLinks; i++)
	{
		PolarField* inputField = (PolarField*)(linkedFields.at(i)->input);
		PolarField* outputField = (PolarField*)(linkedFields.at(i)->output);

		inputFields << inputField->getXcoord() << " " << inputField->getYcoord() << " "
					<< inputField->getRadius() << " " << inputField->getUsage() << "\n";

		outputFields << outputField->getXcoord() << " " << outputField->getYcoord() << " "
					<< outputField->getRadius() << " " << outputField->getUsage() << "\n";

		combined <<  inputField->getXcoord() << " " << inputField->getYcoord() << " " << inputField->getRadius() << " "
				 << outputField->getXcoord() << " " << outputField->getYcoord() << " " << outputField->getRadius() << " "
				 << linkedFields.at(i)->getUsage() << "\n";
	}

	inputFields.close();
	outputFields.close();
	combined.close();



	ppm = io.loadMappingFromXML(path + "torso_"+filename+".xml");
//	ppm->printLinkedFields();

	numLinks = ppm->getNumLinks();
	linkedFields = ppm->getLinkedFields();

	fullname = path + "torso_inputFields.txt";
	inputFields.open(fullname.c_str());
	inputFields << "x y radius usage\n";

	fullname = path + "torso_outputFields.txt";
	outputFields.open(fullname.c_str());
	outputFields << "x y radius usage\n";

	fullname = path + "torso_links.txt";
	combined.open(fullname.c_str());
	combined << "inpX inpY inpRad outpX outpY outpRad lnkUsg\n";

	for(size_t i=0; i<numLinks; i++)
	{
		PolarField* inputField = (PolarField*)(linkedFields.at(i)->input);
		PolarField* outputField = (PolarField*)(linkedFields.at(i)->output);

		inputFields << inputField->getXcoord() << " " << inputField->getYcoord() << " "
					<< inputField->getRadius() << " " << inputField->getUsage() << "\n";

		outputFields << outputField->getXcoord() << " " << outputField->getYcoord() << " "
					<< outputField->getRadius() << " " << outputField->getUsage() << "\n";

		combined <<  inputField->getXcoord() << " " << inputField->getYcoord() << " " << inputField->getRadius() << " "
				 << outputField->getXcoord() << " " << outputField->getYcoord() << " " << outputField->getRadius() << " "
				 << linkedFields.at(i)->getUsage() << "\n";
	}

	inputFields.close();
	outputFields.close();
	combined.close();


	delete ppm;
	return (0);

}

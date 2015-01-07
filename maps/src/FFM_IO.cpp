	/*
 * FFM_IO.cpp
 *
 *  Created on: 21 Oct 2010
 *      Author: phs
 */

#include <string>
#include <vector>

#include "FFM_IO.h"


FFM_IO::FFM_IO() {

}

FFM_IO::~FFM_IO() {
}


bool FFM_IO::saveMappingToXML(FieldFieldMapping *ffm, string filename){
	CXmlMap *cxm = new CXmlMap();
	cxm->init(4,4,1,ffm->getNumLinks());


	//Comment describes details that are required to reproduce the full field maps
	//XML then contains the details about the fields which are linked.

	string inputDimensions ("[");
	inputDimensions += NumberToString<int>(ffm->getInputFieldType()) + ",";
	inputDimensions += NumberToString<float>(ffm->getInputMinX()) + ",";
	inputDimensions += NumberToString<float>(ffm->getInputMaxX()) + ",";
	inputDimensions += NumberToString<float>(ffm->getInputMinY()) + ",";
	inputDimensions += NumberToString<float>(ffm->getInputMaxY()) + "]";

	string outputDimensions ("[");
	outputDimensions += NumberToString<int>(ffm->getOutputFieldType()) + ",";
	outputDimensions += NumberToString<float>(ffm->getOutputMinX()) + ",";
	outputDimensions += NumberToString<float>(ffm->getOutputMaxX()) + ",";
	outputDimensions += NumberToString<float>(ffm->getOutputMinY()) + ",";
	outputDimensions += NumberToString<float>(ffm->getOutputMaxY()) + "]";

	string comment = inputDimensions + outputDimensions;

	float *input = new float[3];
	float *output = new float[3];
	float *atts = new float[1];
	vector<FieldLink*> linkedFields = ffm->getLinkedFields();
	for(size_t i=0; i<linkedFields.size(); i++)
	{
		FieldLink* l = linkedFields.at(i);
		Field* temp = l->input;
		input[0] = temp->getXcoord();
		input[1] = temp->getYcoord();
		input[2] = temp->getCalcdLinks();
		input[3] = temp->getUsage();

		temp = l->output;
		output[0] = temp->getXcoord();
		output[1] = temp->getYcoord();
		output[2] = temp->getCalcdLinks();
		output[3] = temp->getUsage();

		atts[0] = l->usage;

		cxm->addLink(input,output,atts);
	}

	char * cFilename;
	cFilename = new char [filename.size()+1];
	strcpy (cFilename, filename.c_str());

	cxm->writeComments(comment);
	cout << comment << endl;
	cxm->writeXmlFile(cFilename);


	return true;
}



FieldFieldMapping* FFM_IO:: loadMappingFromXML(string filename){
	//Can use the following to break down the comment into usable data again
	//find(char, pos0)
	//substr...
	/*
	 * template <typename T>
  T StringToNumber ( const string &Text )
  {
     istringstream ss(Text);
     T result;
     return ss >> result ? result : 0;
  }
  */
	cout << "Attempting to load data from: " << filename << endl;
	CXmlMap *cxm = new CXmlMap();
	cxm->init(4,4,1,10);

	char * cFilename;
	cFilename = new char [filename.size()+1];
	strcpy (cFilename, filename.c_str());

	cxm->readXmlFile(cFilename);
	string comments = cxm->getComments();
	/*
	 * Format of comments string should be:
	 * [inputType,inputMinX, inputMaxX, inputMinY, inputMaxY][outputType,outputMinX, outputMaxX, outputMinY, outputMaxY]
	 */
	int iit, iot;
	float iimix,iimax,iimiy,iimay,iomix,iomax,iomiy,iomay;

	if(comments == "")
	{
		cout << "Unable to read dimensions from file, using default dimensions" << endl;
		iit=1; iot=1;
		iimix=0; iimax=320; iimiy=0; iimay=240;
		iomix=-60;iomax=60;iomiy=-53;iomay=53;
	}
	else
	{
		cout << "Reading dimensions from file" << endl;
		int start = comments.find('[');
		int end = comments.find(']');
		string input = comments.substr(start+1, end-start-1);	//remove brackets at either end

		start = comments.find('[',end);
		end = comments.find(']',start);
		string output = comments.substr(start+1, end-start-1);

		//Tokenize input string
		start = 0;
		end = input.find(',');
		string inputType = input.substr(0,end);
		iit = StringToNumber<int>(inputType);

		start = end+1;
		end = input.find(',', start);
		string inputMinX = input.substr(start,end-start);
		iimix = StringToNumber<float>(inputMinX);

		start = end+1;
		end = input.find(',', start);
		string inputMaxX = input.substr(start,end-start);
		iimax = StringToNumber<float>(inputMaxX);

		start = end+1;
		end = input.find(',', start);
		string inputMinY = input.substr(start,end-start);
		iimiy = StringToNumber<float>(inputMinY);

		start = end+1;
		string inputMaxY = input.substr(start);
		iimay = StringToNumber<float>(inputMaxY);

		//Tokenize output string
		start = 0;
		end = output.find(',');
		string outputType = output.substr(0,end);
		iot = StringToNumber<int>(outputType);

		start = end+1;
		end = output.find(',', start);
		string outputMinX = output.substr(start,end-start);
		iomix = StringToNumber<float>(outputMinX);

		start = end+1;
		end = output.find(',', start);
		string outputMaxX = output.substr(start,end-start);
		iomax = StringToNumber<float>(outputMaxX);

		start = end+1;
		end = output.find(',', start);
		string outputMinY = output.substr(start,end-start);
		iomiy = StringToNumber<float>(outputMinY);

		start = end+1;
		string outputMaxY = output.substr(start);
		iomay = StringToNumber<float>(outputMaxY);
	}

#ifdef DEBUG
	cout <<"iit:" << iit << ", iimix:"<< iimix <<", iimax:"<< iimax <<", iimiy:" << iimiy << ", iimay:"<< iimay
			<< ", iot: "<< iot <<", iomix:"<< iomix << ", iomax:"<< iomax << ", iomiy:" << iomiy << ", iomay:" << iomay << endl;
#endif
	FieldFieldMapping* ffm = new FieldFieldMapping(iit,iimix,iimax,iimiy,iimay,
													iot,iomix,iomax,iomiy,iomay);

#ifdef DEBUG
	cout << "Generated new map, with correct dimensions, now to add data" << endl;
	cout << "Num links to load: " << cxm->nmbOfLinks() << endl;
#endif

	float* link = new float[9];
	for(int i=0; i<cxm->nmbOfLinks();i++)
	{
#ifdef DEBUG
		cout << link[0] << ", " << link[1] << ", " << link[2] << ", " << link[3] << ", " << link[4] << ", "
				<< link[5] << ", " << link[6] << ", " << link[7] << ", " << link[8] << endl;
#endif
		link = cxm->ptrLinkByIndex(i);
		/*
		 * Reminder from saving of what the different indexes contain.
		 * input[0] = temp->getXcoord();		//0
		 * input[1] = temp->getYcoord();		//1
		 * input[2] = temp->getCalcdLinks();	//2
		 * input[3] = temp->getUsage();			//3

		 * output[0] = temp->getXcoord();		//4
		 * output[1] = temp->getYcoord();		//5
		 * output[2] = temp->getCalcdLinks();	//6
		 * output[3] = temp->getUsage();		//7

		 * atts[0] = 1.f;
		 */
//		cout << link[5] << endl;
		bool success = ffm->addLink(link[0],link[1],link[4],link[5],link[2]);
		if(success)
		{
			if(link[2] != link[6])
			{
				Field* field = ffm->getOutputField(link[4],link[5]);
				field->setCalcdLinks(link[6]);
			}
			Field* input = ffm->getInputField(link[0],link[1]);
			input->setUsage(link[3]);
			Field* output = ffm->getOutputField(link[4],link[5]);
			output->setUsage(link[7]);
			FieldLink* linkedFields = ffm->getLink(input, output);
			linkedFields->usage = link[8];
#ifdef DEBUG
			cout << i <<": " << linkedFields << endl;
#endif
		}
		else
		{
#ifdef DEBUG
			cout << "Fields didn't exist, unable to add link" << endl;
#endif
		}

	}
//	cout << "Returning from FFM_IO" << endl;
	return ffm;
}

